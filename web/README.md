# WT_MPC Telemetry Web

Local web dashboard and JSON API for MuJoCo/MPC tuning telemetry.

The dashboard uses WebSocket for live updates:

```text
Browser <--> ws://127.0.0.1:8765/ws
```

Your C++ controller can still send telemetry with HTTP:

```text
C++ -> POST http://127.0.0.1:8765/api/telemetry
server -> WebSocket broadcast -> browser
```

This keeps the controller side simple and lets the browser update immediately without polling.

## Run

```bash
python3 web/server.py --host 127.0.0.1 --port 8765
```

Open:

```text
http://127.0.0.1:8765
```

## API

### POST `/api/telemetry`

Send one sample:

```json
{
  "sim_time": 1.234,
  "imu": {
    "accel": [0.0, 0.0, -9.81],
    "gyro": [0.0, 0.0, 0.0],
    "quat": [1.0, 0.0, 0.0, 0.0],
    "rpy": [0.0, 0.0, 0.0]
  },
  "com": {
    "pos": [0.1, 0.0, 0.25]
  },
  "params": {
    "kp": 14.0,
    "kd": 1.2
  },
  "state": "stand"
}
```

Send a batch:

```json
{
  "samples": [
    {
      "sim_time": 1.234,
      "imu": {
        "accel": [0.0, 0.0, -9.81],
        "gyro": [0.0, 0.0, 0.0],
        "quat": [1.0, 0.0, 0.0, 0.0],
        "rpy": [0.0, 0.0, 0.0]
      },
      "com": {
        "pos": [0.1, 0.0, 0.25]
      }
    }
  ]
}
```

`imu.rpy` uses radians in `[roll, pitch, yaw]` order. Aliases are accepted: `accelerometer`, `gyroscope`, `quaternion`, `accel`, `gyro`, `quat`, and top-level `rpy`.

### GET `/api/telemetry?limit=500`

Returns recent samples.

### GET `/api/export?format=csv`

Exports currently retained samples as CSV.

### DELETE `/api/telemetry`

Clears in-memory samples and the JSONL log.

Data is appended to `web/data/telemetry.jsonl`.

## Online debug params

The dashboard has a **Trotting 在线参数** panel for tuning the six Trotting gain groups. The web server stores the latest values in:

```text
web/data/debug_params.json
```

Read current values:

```bash
curl http://127.0.0.1:8765/api/debug-params
```

Apply values:

```bash
curl -X POST http://127.0.0.1:8765/api/debug-params \
  -H 'Content-Type: application/json' \
  -d '{
    "params": {
      "trotting": {
        "Kpp": [45.0, 45.0, 45.0],
        "Kdp": [5.0, 5.0, 5.0],
        "kpw": 230.0,
        "Kdw": [22.0, 22.0, 22.0],
        "KpSwing": [90.0, 90.0, 90.0],
        "KdSwing": [2.0, 2.0, 2.0]
      }
    }
  }'
```

Reset to defaults:

```bash
curl -X DELETE http://127.0.0.1:8765/api/debug-params
```

The server also broadcasts updated params over WebSocket:

```json
{"type":"debug_params","params":{...}}
```

## WebSocket messages

Connect the browser or a client to:

```text
ws://127.0.0.1:8765/ws
```

Server messages:

```json
{"type":"init","samples":[...]}
```

```json
{"type":"samples","samples":[...]}
```

```json
{"type":"clear"}
```

For a high-rate robot pipeline, prefer batching samples on the C++ side and POSTing 20-100 Hz batches instead of sending one HTTP request per control step.
