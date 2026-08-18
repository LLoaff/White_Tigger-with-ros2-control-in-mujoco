const state = {
  paused: false,
  ws: null,
  wsConnected: false,
  samples: [],
  maxSamples: 20000,
  com: {
    followLatest: true,
    viewEnd: null,
    viewSpan: 12,
    drag: null,
  },
};

const colors = {
  w: "#1b1f29",
  x: "#1464d2",
  y: "#14925f",
  z: "#c64b4b",
};

const $ = (id) => document.getElementById(id);
const chartPad = { left: 46, right: 14, top: 12, bottom: 28 };

const comAxes = [
  {
    key: "x",
    label: "X",
    color: colors.x,
    canvasId: "comXChart",
    minId: "comYMinX",
    maxId: "comYMaxX",
    autoId: "comYAutoX",
    get: (s) => s.com?.pos?.[0],
  },
  {
    key: "y",
    label: "Y",
    color: colors.y,
    canvasId: "comYChart",
    minId: "comYMinY",
    maxId: "comYMaxY",
    autoId: "comYAutoY",
    get: (s) => s.com?.pos?.[1],
  },
  {
    key: "z",
    label: "Z",
    color: colors.z,
    canvasId: "comZChart",
    minId: "comYMinZ",
    maxId: "comYMaxZ",
    autoId: "comYAutoZ",
    get: (s) => s.com?.pos?.[2],
  },
];

function fmt(value, digits = 3) {
  return typeof value === "number" && Number.isFinite(value) ? value.toFixed(digits) : "-";
}

function sampleTime(sample) {
  return typeof sample.sim_time === "number" ? sample.sim_time : sample.received_at;
}

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function readComWindowSeconds() {
  const input = $("comWindowSeconds");
  const value = Number(input?.value);
  if (Number.isFinite(value) && value > 0) {
    state.com.viewSpan = value;
  }
  return state.com.viewSpan;
}

function getTimedSamples(samples) {
  return samples.filter((sample) => Number.isFinite(sampleTime(sample)));
}

function getComWindow(samples) {
  const timed = getTimedSamples(samples);
  if (timed.length < 2) return null;

  const times = timed.map(sampleTime);
  const earliest = Math.min(...times);
  const latest = Math.max(...times);
  const fullSpan = Math.max(0, latest - earliest);
  const requestedSpan = readComWindowSeconds();
  const span = Math.max(1e-9, Math.min(requestedSpan, fullSpan || requestedSpan));

  if (fullSpan <= span) {
    return { start: earliest, end: latest, span, earliest, latest, fullSpan };
  }

  let end = state.com.followLatest || !Number.isFinite(state.com.viewEnd) ? latest : state.com.viewEnd;
  end = clamp(end, earliest + span, latest);
  state.com.viewEnd = end;
  return { start: end - span, end, span, earliest, latest, fullSpan };
}

function readYRange(axis) {
  const min = Number($(axis.minId).value);
  const max = Number($(axis.maxId).value);
  return Number.isFinite(min) && Number.isFinite(max) && max > min ? [min, max] : null;
}

function normalizeChartOptions(options) {
  if (Array.isArray(options)) return { fixedRange: options };
  return options || {};
}

function drawChart(canvas, samples, series, chartOptions) {
  const options = normalizeChartOptions(chartOptions);
  const ctx = canvas.getContext("2d");
  const width = canvas.width;
  const height = canvas.height;
  ctx.clearRect(0, 0, width, height);

  const pad = chartPad;
  const plotW = width - pad.left - pad.right;
  const plotH = height - pad.top - pad.bottom;

  ctx.fillStyle = "#fbfcfe";
  ctx.fillRect(0, 0, width, height);
  ctx.strokeStyle = "#d9e0e8";
  ctx.lineWidth = 1;

  for (let i = 0; i <= 4; i++) {
    const y = pad.top + (plotH * i) / 4;
    ctx.beginPath();
    ctx.moveTo(pad.left, y);
    ctx.lineTo(width - pad.right, y);
    ctx.stroke();
  }

  const timedSamples = getTimedSamples(samples);
  const timeRange = Array.isArray(options.timeRange) ? options.timeRange : null;
  const t0 = timeRange ? timeRange[0] : sampleTime(timedSamples[0]);
  const t1 = timeRange ? timeRange[1] : sampleTime(timedSamples[timedSamples.length - 1]);
  const visibleSamples = timedSamples.filter((sample) => {
    const t = sampleTime(sample);
    return !timeRange || (t >= t0 && t <= t1);
  });

  if (timeRange) {
    ctx.fillStyle = "#657184";
    ctx.font = "11px sans-serif";
    ctx.fillText(fmt(t0, 2), pad.left, height - 8);
    ctx.fillText(fmt(t1, 2), width - 70, height - 8);
  }

  if (visibleSamples.length < 2) {
    ctx.fillStyle = "#657184";
    ctx.font = "14px sans-serif";
    ctx.fillText(samples.length < 2 ? "waiting for telemetry" : "no data in window", pad.left, pad.top + 28);
    return;
  }

  let values = [];
  for (const sample of visibleSamples) {
    for (const item of series) {
      const value = item.get(sample);
      if (typeof value === "number" && Number.isFinite(value)) values.push(value);
    }
  }
  if (!values.length) return;

  let min = Math.min(...values);
  let max = Math.max(...values);
  if (options.fixedRange) {
    min = options.fixedRange[0];
    max = options.fixedRange[1];
  }
  if (Math.abs(max - min) < 1e-9) {
    min -= 1;
    max += 1;
  }
  if (!options.fixedRange) {
    const margin = (max - min) * 0.08;
    min -= margin;
    max += margin;
  }

  const timeSpan = Math.max(1e-9, t1 - t0);

  ctx.fillStyle = "#657184";
  ctx.font = "11px sans-serif";
  ctx.fillText(max.toFixed(2), 8, pad.top + 4);
  ctx.fillText(min.toFixed(2), 8, height - pad.bottom);
  if (!timeRange) {
    ctx.fillText(fmt(t0, 2), pad.left, height - 8);
    ctx.fillText(fmt(t1, 2), width - 70, height - 8);
  }

  for (const item of series) {
    ctx.strokeStyle = item.color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    let started = false;
    for (const sample of visibleSamples) {
      const value = item.get(sample);
      if (typeof value !== "number" || !Number.isFinite(value)) continue;
      const x = pad.left + ((sampleTime(sample) - t0) / timeSpan) * plotW;
      const y = pad.top + (1 - (value - min) / (max - min)) * plotH;
      if (!started) {
        ctx.moveTo(x, y);
        started = true;
      } else {
        ctx.lineTo(x, y);
      }
    }
    ctx.stroke();
  }
}

function renderComCharts(samples) {
  const windowRange = getComWindow(samples);
  const timeRange = windowRange ? [windowRange.start, windowRange.end] : null;
  for (const axis of comAxes) {
    drawChart($(axis.canvasId), samples, [
      { color: axis.color, get: axis.get },
    ], {
      fixedRange: readYRange(axis),
      timeRange,
    });
  }
}

function updateMetrics(samples) {
  $("sampleCount").textContent = samples.length;
  const latest = samples[samples.length - 1];
  if (!latest) return;
  $("simTime").textContent = fmt(latest.sim_time, 4);
  $("stateValue").textContent = latest.state || latest.fsm_mode || "-";
  $("receivedAt").textContent = latest.received_at
    ? new Date(latest.received_at * 1000).toLocaleTimeString()
    : "-";
  $("latestPayload").textContent = JSON.stringify(latest, null, 2);
}

function render() {
  const samples = state.samples.slice(-state.maxSamples);
  updateMetrics(samples);
  drawChart($("accelChart"), samples, [
    { color: colors.x, get: (s) => s.imu?.accel?.[0] },
    { color: colors.y, get: (s) => s.imu?.accel?.[1] },
    { color: colors.z, get: (s) => s.imu?.accel?.[2] },
  ]);
  drawChart($("gyroChart"), samples, [
    { color: colors.x, get: (s) => s.imu?.gyro?.[0] },
    { color: colors.y, get: (s) => s.imu?.gyro?.[1] },
    { color: colors.z, get: (s) => s.imu?.gyro?.[2] },
  ]);
  renderComCharts(samples);
  drawChart($("quatChart"), samples, [
    { color: colors.w, get: (s) => s.imu?.quat?.[0] },
    { color: colors.x, get: (s) => s.imu?.quat?.[1] },
    { color: colors.y, get: (s) => s.imu?.quat?.[2] },
    { color: colors.z, get: (s) => s.imu?.quat?.[3] },
  ], [-1, 1]);
}

async function fetchSamples() {
  if (state.paused || state.wsConnected) return;
  try {
    const res = await fetch(`/api/telemetry?limit=${state.maxSamples}`);
    const data = await res.json();
    state.samples = data.samples || [];
    $("status").textContent = `Connected · ${new Date().toLocaleTimeString()}`;
    render();
  } catch (err) {
    $("status").textContent = `Disconnected · ${err.message}`;
  }
}

function appendSamples(samples) {
  if (!Array.isArray(samples) || state.paused) return;
  state.samples.push(...samples);
  if (state.samples.length > state.maxSamples) {
    state.samples = state.samples.slice(-state.maxSamples);
  }
  render();
}

function finishComDrag(pointerId) {
  const drag = state.com.drag;
  if (!drag || drag.pointerId !== pointerId) return;
  for (const axis of comAxes) {
    $(axis.canvasId).classList.remove("dragging");
  }
  state.com.drag = null;
}

function setupComPan(canvas) {
  canvas.addEventListener("pointerdown", (event) => {
    const windowRange = getComWindow(state.samples);
    if (!windowRange || windowRange.fullSpan <= windowRange.span) return;

    const rect = canvas.getBoundingClientRect();
    state.com.drag = {
      pointerId: event.pointerId,
      startX: event.clientX,
      startEnd: windowRange.end,
      span: windowRange.span,
      earliest: windowRange.earliest,
      latest: windowRange.latest,
      plotPixels: Math.max(1, rect.width - chartPad.left - chartPad.right),
    };
    state.com.followLatest = false;
    canvas.classList.add("dragging");
    canvas.setPointerCapture(event.pointerId);
  });

  canvas.addEventListener("pointermove", (event) => {
    const drag = state.com.drag;
    if (!drag || drag.pointerId !== event.pointerId) return;

    const deltaX = event.clientX - drag.startX;
    const secondsPerPixel = drag.span / drag.plotPixels;
    const minEnd = drag.earliest + drag.span;
    const nextEnd = clamp(drag.startEnd - deltaX * secondsPerPixel, minEnd, drag.latest);
    state.com.viewEnd = nextEnd;
    state.com.followLatest = nextEnd >= drag.latest - 1e-6;
    render();
  });

  canvas.addEventListener("pointerup", (event) => finishComDrag(event.pointerId));
  canvas.addEventListener("pointercancel", (event) => finishComDrag(event.pointerId));
  canvas.addEventListener("dblclick", () => {
    state.com.followLatest = true;
    state.com.viewEnd = null;
    render();
  });
}

function setupComControls() {
  $("comLatestBtn").addEventListener("click", () => {
    state.com.followLatest = true;
    state.com.viewEnd = null;
    render();
  });

  $("comWindowSeconds").addEventListener("change", () => {
    readComWindowSeconds();
    if (state.com.followLatest) state.com.viewEnd = null;
    render();
  });

  for (const axis of comAxes) {
    $(axis.minId).addEventListener("input", render);
    $(axis.maxId).addEventListener("input", render);
    $(axis.autoId).addEventListener("click", () => {
      $(axis.minId).value = "";
      $(axis.maxId).value = "";
      render();
    });
    setupComPan($(axis.canvasId));
  }
}

function connectWebSocket() {
  if (!("WebSocket" in window)) {
    $("status").textContent = "WebSocket unavailable · using HTTP polling";
    return;
  }
  const scheme = window.location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${scheme}://${window.location.host}/ws`);
  state.ws = ws;

  ws.addEventListener("open", () => {
    state.wsConnected = true;
    $("status").textContent = `WebSocket connected · ${new Date().toLocaleTimeString()}`;
  });

  ws.addEventListener("message", (event) => {
    const message = JSON.parse(event.data);
    if (message.type === "init") {
      state.samples = message.samples || [];
      render();
    } else if (message.type === "samples") {
      appendSamples(message.samples);
    } else if (message.type === "clear") {
      state.samples = [];
      render();
    }
  });

  ws.addEventListener("close", () => {
    state.wsConnected = false;
    $("status").textContent = "WebSocket closed · using HTTP polling";
    setTimeout(connectWebSocket, 1500);
  });

  ws.addEventListener("error", () => {
    state.wsConnected = false;
    $("status").textContent = "WebSocket error · using HTTP polling";
    ws.close();
  });
}

async function sendSample() {
  const t = performance.now() / 1000;
  await fetch("/api/telemetry", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      sim_time: t,
      imu: {
        accel: [Math.sin(t) * 0.4, Math.cos(t * 0.6) * 0.3, -9.81 + Math.sin(t * 1.8) * 0.2],
        gyro: [Math.sin(t * 1.4) * 0.05, Math.cos(t * 1.2) * 0.04, Math.sin(t * 0.7) * 0.1],
        quat: [1, Math.sin(t) * 0.02, Math.cos(t) * 0.02, Math.sin(t * 0.5) * 0.03],
      },
      com: {
        pos: [Math.sin(t * 0.4) * 0.1, Math.cos(t * 0.3) * 0.05, 0.25 + Math.sin(t * 0.5) * 0.02],
      },
      params: { kp: 14, kd: 1.2 },
      state: "demo",
    }),
  });
  fetchSamples();
}

$("pauseBtn").addEventListener("click", () => {
  state.paused = !state.paused;
  $("pauseBtn").textContent = state.paused ? "Resume" : "Pause";
});
$("sampleBtn").addEventListener("click", sendSample);
setupComControls();

connectWebSocket();
fetchSamples();
setInterval(fetchSamples, 500);
