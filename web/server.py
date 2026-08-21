#!/usr/bin/env python3
import argparse
import base64
import csv
import hashlib
import json
import mimetypes
import os
import struct
import threading
import time
from collections import deque
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from pathlib import Path
from urllib.parse import parse_qs, urlparse


ROOT = Path(__file__).resolve().parent
PUBLIC_DIR = ROOT / "public"
DATA_DIR = ROOT / "data"
LOG_FILE = DATA_DIR / "telemetry.jsonl"
DEBUG_PARAMS_FILE = DATA_DIR / "debug_params.json"
MAX_MEMORY_SAMPLES = 20000

SAMPLES = deque(maxlen=MAX_MEMORY_SAMPLES)
NEXT_ID = 1
SAMPLE_LOCK = threading.Lock()
PARAM_LOCK = threading.Lock()
WS_CLIENTS = {}
WS_LOCK = threading.Lock()
WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

DEFAULT_DEBUG_PARAMS = {
    "joint_gains": {
        "swing": {
            "go1": {"kp": 5.5, "kd": 1.0},
            "white_tigger": {"kp": 2.5, "kd": 0.3},
        },
        "stable": {
            "go1": {"kp": 7.0, "kd": 2.0},
            "white_tigger": {"kp": 3.5, "kd": 1.1},
        },
    },
    "trotting": {
        "Kpp": [45.0, 45.0, 45.0],
        "Kdp": [5.0, 5.0, 5.0],
        "kpw": 230.0,
        "Kdw": [22.0, 22.0, 22.0],
        "KpSwing": [90.0, 90.0, 90.0],
        "KdSwing": [2.0, 2.0, 2.0],
    },
}

DEBUG_PARAMS = {}


def as_float_list(value, length):
    if value is None:
        return None
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f"expected list with {length} numbers")
    out = []
    for item in value:
        if not isinstance(item, (int, float)):
            raise ValueError("list values must be numbers")
        out.append(float(item))
    return out


def clone_json(value):
    return json.loads(json.dumps(value))


def merge_dict(base, updates):
    out = clone_json(base)
    if not isinstance(updates, dict):
        return out
    for key, value in updates.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = merge_dict(out[key], value)
        else:
            out[key] = value
    return out


def require_number(value, path):
    if not isinstance(value, (int, float)):
        raise ValueError(f"{path} must be a number")
    return float(value)


def validate_gain_pair(value, path):
    if not isinstance(value, dict):
        raise ValueError(f"{path} must be an object")
    return {
        "kp": require_number(value.get("kp"), f"{path}.kp"),
        "kd": require_number(value.get("kd"), f"{path}.kd"),
    }


def validate_vec3(value, path):
    return as_float_list(value, 3)


def validate_debug_params(raw):
    if not isinstance(raw, dict):
        raise ValueError("debug params must be a JSON object")

    merged = merge_dict(DEFAULT_DEBUG_PARAMS, raw)
    joint = merged.get("joint_gains")
    trotting = merged.get("trotting")
    if not isinstance(joint, dict):
        raise ValueError("joint_gains must be an object")
    if not isinstance(trotting, dict):
        raise ValueError("trotting must be an object")

    clean = clone_json(DEFAULT_DEBUG_PARAMS)
    for phase in ("swing", "stable"):
        phase_value = joint.get(phase)
        if not isinstance(phase_value, dict):
            raise ValueError(f"joint_gains.{phase} must be an object")
        for model_name in ("go1", "white_tigger"):
            clean["joint_gains"][phase][model_name] = validate_gain_pair(
                phase_value.get(model_name),
                f"joint_gains.{phase}.{model_name}",
            )

    for name in ("Kpp", "Kdp", "Kdw", "KpSwing", "KdSwing"):
        clean["trotting"][name] = validate_vec3(trotting.get(name), f"trotting.{name}")
    clean["trotting"]["kpw"] = require_number(trotting.get("kpw"), "trotting.kpw")
    clean["updated_at"] = time.time()
    return clean


def load_debug_params():
    global DEBUG_PARAMS
    if DEBUG_PARAMS_FILE.exists():
        try:
            loaded = json.loads(DEBUG_PARAMS_FILE.read_text(encoding="utf-8"))
            DEBUG_PARAMS = validate_debug_params(loaded)
            return
        except (OSError, json.JSONDecodeError, ValueError) as exc:
            print(f"Failed to load debug params, using defaults: {exc}")
    DEBUG_PARAMS = validate_debug_params(DEFAULT_DEBUG_PARAMS)


def store_debug_params(params):
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    DEBUG_PARAMS_FILE.write_text(
        json.dumps(params, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def get_debug_params():
    with PARAM_LOCK:
        return clone_json(DEBUG_PARAMS)


def pick(raw, *paths):
    for path in paths:
        cur = raw
        ok = True
        for key in path:
            if not isinstance(cur, dict) or key not in cur:
                ok = False
                break
            cur = cur[key]
        if ok:
            return cur
    return None


def normalize_sample(raw):
    if not isinstance(raw, dict):
        raise ValueError("sample must be a JSON object")

    imu = raw.get("imu") if isinstance(raw.get("imu"), dict) else {}
    sample = {
        "id": None,
        "received_at": time.time(),
        "source": raw.get("source", "mujoco"),
        "sim_time": raw.get("sim_time", raw.get("time")),
        "fsm_mode": raw.get("fsm_mode"),
        "state": raw.get("state"),
        "imu": {
            "accel": as_float_list(pick(raw, ("imu", "accel"), ("imu", "accelerometer"), ("accel",), ("accelerometer",)), 3),
            "gyro": as_float_list(pick(raw, ("imu", "gyro"), ("imu", "gyroscope"), ("gyro",), ("gyroscope",)), 3),
            "quat": as_float_list(pick(raw, ("imu", "quat"), ("imu", "quaternion"), ("quat",), ("quaternion",)), 4),
            "rpy": as_float_list(pick(raw, ("imu", "rpy"), ("rpy",)), 3),
        },
        "com": {
            "pos": as_float_list(pick(raw, ("com", "pos"), ("com_pos",), ("pcom",)), 3),
        },
        "params": raw.get("params", {}),
        "raw": raw,
    }

    if sample["sim_time"] is not None:
        if not isinstance(sample["sim_time"], (int, float)):
            raise ValueError("sim_time/time must be a number")
        sample["sim_time"] = float(sample["sim_time"])

    if sample["params"] is None:
        sample["params"] = {}
    if not isinstance(sample["params"], dict):
        raise ValueError("params must be an object")

    if raw.get("id") is not None:
        sample["source_id"] = raw.get("id")
    if imu.get("temperature") is not None:
        sample["imu"]["temperature"] = imu.get("temperature")

    return sample


def load_existing():
    global NEXT_ID
    if not LOG_FILE.exists():
        return
    with LOG_FILE.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                sample = json.loads(line)
            except json.JSONDecodeError:
                continue
            with SAMPLE_LOCK:
                SAMPLES.append(sample)
                if isinstance(sample.get("id"), int):
                    NEXT_ID = max(NEXT_ID, sample["id"] + 1)


def store_samples(samples):
    global NEXT_ID
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    with SAMPLE_LOCK:
        with LOG_FILE.open("a", encoding="utf-8") as f:
            for sample in samples:
                sample["id"] = NEXT_ID
                NEXT_ID += 1
                SAMPLES.append(sample)
                f.write(json.dumps(sample, ensure_ascii=False, separators=(",", ":")) + "\n")


def websocket_frame(payload):
    data = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    header = bytearray([0x81])
    length = len(data)
    if length < 126:
        header.append(length)
    elif length < 65536:
        header.append(126)
        header.extend(struct.pack("!H", length))
    else:
        header.append(127)
        header.extend(struct.pack("!Q", length))
    return bytes(header) + data


def send_ws(conn, payload):
    frame = websocket_frame(payload)
    with WS_LOCK:
        lock = WS_CLIENTS.get(conn)
    if lock is None:
        return False
    try:
        with lock:
            conn.sendall(frame)
        return True
    except OSError:
        remove_ws_client(conn)
        return False


def broadcast_ws(payload):
    with WS_LOCK:
        clients = list(WS_CLIENTS.keys())
    for conn in clients:
        send_ws(conn, payload)


def remove_ws_client(conn):
    with WS_LOCK:
        WS_CLIENTS.pop(conn, None)
    try:
        conn.shutdown(2)
    except OSError:
        pass
    try:
        conn.close()
    except OSError:
        pass


def recv_ws_frame(conn):
    header = conn.recv(2)
    if not header:
        return None, None
    first, second = header
    opcode = first & 0x0F
    masked = bool(second & 0x80)
    length = second & 0x7F
    if length == 126:
        length = struct.unpack("!H", conn.recv(2))[0]
    elif length == 127:
        length = struct.unpack("!Q", conn.recv(8))[0]
    mask = conn.recv(4) if masked else b""
    payload = b""
    while len(payload) < length:
        chunk = conn.recv(length - len(payload))
        if not chunk:
            break
        payload += chunk
    if masked:
        payload = bytes(byte ^ mask[i % 4] for i, byte in enumerate(payload))
    return opcode, payload


def flatten_for_csv(sample):
    imu = sample.get("imu", {})
    accel = imu.get("accel") or [None, None, None]
    gyro = imu.get("gyro") or [None, None, None]
    quat = imu.get("quat") or [None, None, None, None]
    rpy = imu.get("rpy") or [None, None, None]
    com = sample.get("com", {})
    com_pos = com.get("pos") or [None, None, None]
    params = sample.get("params") or {}
    return {
        "id": sample.get("id"),
        "received_at": sample.get("received_at"),
        "sim_time": sample.get("sim_time"),
        "source": sample.get("source"),
        "state": sample.get("state"),
        "fsm_mode": sample.get("fsm_mode"),
        "accel_x": accel[0],
        "accel_y": accel[1],
        "accel_z": accel[2],
        "gyro_x": gyro[0],
        "gyro_y": gyro[1],
        "gyro_z": gyro[2],
        "quat_w": quat[0],
        "quat_x": quat[1],
        "quat_y": quat[2],
        "quat_z": quat[3],
        "roll": rpy[0],
        "pitch": rpy[1],
        "yaw": rpy[2],
        "com_x": com_pos[0],
        "com_y": com_pos[1],
        "com_z": com_pos[2],
        "params_json": json.dumps(params, ensure_ascii=False, separators=(",", ":")),
    }


class Handler(BaseHTTPRequestHandler):
    server_version = "WTTelemetry/0.1"

    def end_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, DELETE, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        super().end_headers()

    def log_message(self, fmt, *args):
        print("[%s] %s" % (self.log_date_time_string(), fmt % args))

    def send_json(self, status, payload):
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def read_json(self):
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0:
            raise ValueError("empty request body")
        body = self.rfile.read(length)
        return json.loads(body.decode("utf-8"))

    def do_OPTIONS(self):
        self.send_response(204)
        self.end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == "/ws":
            self.handle_websocket()
            return
        if parsed.path == "/api/health":
            with SAMPLE_LOCK:
                sample_count = len(SAMPLES)
            with WS_LOCK:
                ws_count = len(WS_CLIENTS)
            self.send_json(200, {
                "ok": True,
                "samples": sample_count,
                "websocket_clients": ws_count,
                "log_file": str(LOG_FILE),
                "debug_params_file": str(DEBUG_PARAMS_FILE),
            })
            return
        if parsed.path == "/api/schema":
            self.send_json(200, {
                "post": "POST /api/telemetry",
                "single_sample": {
                    "sim_time": 1.234,
                    "imu": {
                        "accel": [0.0, 0.0, -9.81],
                        "gyro": [0.0, 0.0, 0.0],
                        "quat": [1.0, 0.0, 0.0, 0.0],
                        "rpy": [0.0, 0.0, 0.0]
                    },
                    "com": {"pos": [0.0, 0.0, 0.25]},
                    "params": {"kp": 14.0, "kd": 1.2},
                    "state": "stand"
                },
                "batch": {"samples": ["same shape as single_sample"]},
                "get": "GET /api/telemetry?limit=500",
                "debug_params": {
                    "get": "GET /api/debug-params",
                    "post": "POST /api/debug-params",
                    "delete": "DELETE /api/debug-params"
                },
                "export": "GET /api/export?format=jsonl or csv"
            })
            return
        if parsed.path == "/api/debug-params":
            self.send_json(200, {"ok": True, "params": get_debug_params(), "defaults": clone_json(DEFAULT_DEBUG_PARAMS)})
            return
        if parsed.path == "/api/telemetry":
            query = parse_qs(parsed.query)
            limit = int(query.get("limit", ["500"])[0])
            limit = max(1, min(limit, MAX_MEMORY_SAMPLES))
            with SAMPLE_LOCK:
                samples = list(SAMPLES)[-limit:]
            self.send_json(200, {"count": len(samples), "samples": samples})
            return
        if parsed.path == "/api/export":
            query = parse_qs(parsed.query)
            fmt = query.get("format", ["jsonl"])[0]
            with SAMPLE_LOCK:
                samples = list(SAMPLES)
            if fmt == "csv":
                rows = [flatten_for_csv(s) for s in samples]
                fieldnames = list(flatten_for_csv({}).keys())
                from io import StringIO
                buf = StringIO()
                writer = csv.DictWriter(buf, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(rows)
                body = buf.getvalue().encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/csv; charset=utf-8")
                self.send_header("Content-Disposition", "attachment; filename=telemetry.csv")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            body = "".join(json.dumps(s, ensure_ascii=False, separators=(",", ":")) + "\n" for s in samples).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/x-ndjson; charset=utf-8")
            self.send_header("Content-Disposition", "attachment; filename=telemetry.jsonl")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self.serve_static(parsed.path)

    def handle_websocket(self):
        if self.headers.get("Upgrade", "").lower() != "websocket":
            self.send_json(400, {"error": "missing websocket upgrade"})
            return
        key = self.headers.get("Sec-WebSocket-Key")
        if not key:
            self.send_json(400, {"error": "missing Sec-WebSocket-Key"})
            return

        accept = base64.b64encode(hashlib.sha1((key + WS_GUID).encode("ascii")).digest()).decode("ascii")
        response = (
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Accept: {accept}\r\n"
            "\r\n"
        )
        self.connection.sendall(response.encode("ascii"))
        self.connection.settimeout(30)
        with WS_LOCK:
            WS_CLIENTS[self.connection] = threading.Lock()
        with SAMPLE_LOCK:
            initial_samples = list(SAMPLES)[-MAX_MEMORY_SAMPLES:]
        send_ws(self.connection, {"type": "init", "samples": initial_samples, "debug_params": get_debug_params()})

        try:
            while True:
                try:
                    opcode, payload = recv_ws_frame(self.connection)
                except TimeoutError:
                    continue
                if opcode is None or opcode == 0x8:
                    break
                if opcode == 0x9:
                    self.connection.sendall(b"\x8a\x00")
                elif opcode == 0x1:
                    try:
                        message = json.loads(payload.decode("utf-8"))
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        continue
                    if message.get("type") == "ping":
                        send_ws(self.connection, {"type": "pong", "time": time.time()})
        finally:
            remove_ws_client(self.connection)

    def do_HEAD(self):
        parsed = urlparse(self.path)
        if parsed.path == "/":
            path = "/index.html"
        else:
            path = parsed.path
        rel = path.lstrip("/")
        file_path = (PUBLIC_DIR / rel).resolve()
        if not str(file_path).startswith(str(PUBLIC_DIR.resolve())) or not file_path.is_file():
            self.send_response(404)
            self.end_headers()
            return
        content_type = mimetypes.guess_type(str(file_path))[0] or "application/octet-stream"
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(file_path.stat().st_size))
        self.end_headers()

    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path == "/api/debug-params":
            try:
                payload = self.read_json()
                params_payload = payload.get("params") if isinstance(payload, dict) and isinstance(payload.get("params"), dict) else payload
                params = validate_debug_params(params_payload)
                with PARAM_LOCK:
                    global DEBUG_PARAMS
                    DEBUG_PARAMS = params
                    store_debug_params(DEBUG_PARAMS)
                broadcast_ws({"type": "debug_params", "params": params})
            except (json.JSONDecodeError, ValueError) as exc:
                self.send_json(400, {"error": str(exc)})
                return
            self.send_json(200, {"ok": True, "params": params})
            return
        if parsed.path != "/api/telemetry":
            self.send_json(404, {"error": "not found"})
            return
        try:
            payload = self.read_json()
            raw_samples = payload.get("samples") if isinstance(payload, dict) and isinstance(payload.get("samples"), list) else payload
            if isinstance(raw_samples, list):
                samples = [normalize_sample(item) for item in raw_samples]
            else:
                samples = [normalize_sample(raw_samples)]
            store_samples(samples)
            broadcast_ws({"type": "samples", "samples": samples})
        except (json.JSONDecodeError, ValueError) as exc:
            self.send_json(400, {"error": str(exc)})
            return
        self.send_json(200, {"ok": True, "accepted": len(samples), "last_id": samples[-1]["id"]})

    def do_DELETE(self):
        parsed = urlparse(self.path)
        if parsed.path == "/api/debug-params":
            params = validate_debug_params(DEFAULT_DEBUG_PARAMS)
            with PARAM_LOCK:
                global DEBUG_PARAMS
                DEBUG_PARAMS = params
                store_debug_params(DEBUG_PARAMS)
            broadcast_ws({"type": "debug_params", "params": params})
            self.send_json(200, {"ok": True, "params": params})
            return
        if parsed.path != "/api/telemetry":
            self.send_json(404, {"error": "not found"})
            return
        with SAMPLE_LOCK:
            SAMPLES.clear()
        if LOG_FILE.exists():
            LOG_FILE.unlink()
        broadcast_ws({"type": "clear"})
        self.send_json(200, {"ok": True, "samples": 0})

    def serve_static(self, path):
        if path == "/":
            path = "/index.html"
        rel = path.lstrip("/")
        file_path = (PUBLIC_DIR / rel).resolve()
        if not str(file_path).startswith(str(PUBLIC_DIR.resolve())) or not file_path.is_file():
            self.send_json(404, {"error": "not found"})
            return
        body = file_path.read_bytes()
        content_type = mimetypes.guess_type(str(file_path))[0] or "application/octet-stream"
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def main():
    parser = argparse.ArgumentParser(description="WT_MPC telemetry dashboard and API")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=8765, type=int)
    args = parser.parse_args()

    DATA_DIR.mkdir(parents=True, exist_ok=True)
    load_existing()
    load_debug_params()
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"Telemetry dashboard: http://{args.host}:{args.port}")
    print(f"API endpoint: http://{args.host}:{args.port}/api/telemetry")
    print(f"Log file: {LOG_FILE}")
    server.serve_forever()


if __name__ == "__main__":
    main()
