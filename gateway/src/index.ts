import express, { type Request, type Response } from "express";
import http from "node:http";
import { Readable } from "node:stream";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { WebSocket, WebSocketServer } from "ws";

type JsonValue = null | boolean | number | string | JsonValue[] | { [k: string]: JsonValue };

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

const PORT = Number(process.env.PORT || 8000);
const PYTHON_EXEC_BASE = (process.env.PYTHON_EXEC_BASE || "http://127.0.0.1:8000").replace(/\/+$/, "");
const PYTHON_EXEC_WS = PYTHON_EXEC_BASE.replace(/^http/, "ws");
const REQUEST_TIMEOUT_MS = Number(process.env.REQUEST_TIMEOUT_MS || 5000);

const repoRoot = path.resolve(__dirname, "..", "..");
const frontendDir = path.join(repoRoot, "frontend");

const app = express();
app.use(express.json({ limit: "1mb" }));

let commandQueue: Promise<void> = Promise.resolve();

function enqueue<T>(fn: () => Promise<T>): Promise<T> {
  const run = commandQueue.then(fn, fn);
  commandQueue = run.then(() => undefined, () => undefined);
  return run;
}

function withTimeout(ms: number): AbortController {
  const controller = new AbortController();
  setTimeout(() => controller.abort(), ms).unref?.();
  return controller;
}

async function fetchWithRetry(url: string, init: RequestInit, retries = 1): Promise<globalThis.Response> {
  let lastError: unknown;
  for (let attempt = 0; attempt <= retries; attempt += 1) {
    try {
      const controller = withTimeout(REQUEST_TIMEOUT_MS);
      return await fetch(url, { ...init, signal: controller.signal });
    } catch (err) {
      lastError = err;
      if (attempt < retries) {
        await new Promise((r) => setTimeout(r, 150 * (attempt + 1)));
      }
    }
  }
  throw lastError;
}

function degradedState(reason: string) {
  return {
    ts: Date.now() / 1000,
    state: "IDLE",
    vacuum_ok: false,
    vacuum_kpa: null,
    retry_count: 0,
    recover_count: 0,
    success_count: 0,
    last_event: "PYTHON_EXEC_OFFLINE",
    log: [
      {
        ts: Date.now() / 1000,
        level: "ERROR",
        code: "PYTHON_EXEC_OFFLINE",
        msg: reason,
      },
    ],
    robot_pose: { x_m: 0, y_m: 0, z_m: 0.3, roll_deg: 0, pitch_deg: 90, yaw_deg: 0 },
    target_pose: { x_m: 0.35, y_m: 0.12, z_m: 0.0 },
    place_pose: { x_m: -0.25, y_m: 0.25, z_m: 0.0 },
    joint_angles_rad: [0, -1.1, 1.6, 0.6, 1.57, 0],
    vision: { detected: false, confidence: 0.0, tag_id: null, source: "wrist_usb", camera_ok: false, pose_valid: false },
    grip: { vacuum_on: false, sealed: false },
    fault: { active: true, code: "PYTHON_EXEC_OFFLINE", msg: reason },
    conveyor: { mode: "real_io", running_cmd: false, running_fb: false, ready: false, last_start_ts: 0, timeout_active: false },
    vacuum: { mode: "simulated_io", pressure_kpa: null, sensor_ok: false },
    robot: { mode: "real_ros2", connected: false, servo_enabled: false, motion_busy: false, estop_latched: false },
    task: { name: "clean_to_conveyor_to_dirty", phase: "WAITING", cycle_id: "", paused: false },
    safety: { web_motion_allowed: false, manual_jog_enabled: false },
    gateway: { connected_to_python: false }
  };
}

async function proxyJson(req: Request, res: Response, targetPath: string, opts?: { retries?: number; queue?: boolean }) {
  const doRequest = async () => {
    try {
      const response = await fetchWithRetry(`${PYTHON_EXEC_BASE}${targetPath}`, {
        method: req.method,
        headers: {
          "content-type": req.headers["content-type"] || "application/json",
        },
        body: req.method === "GET" ? undefined : JSON.stringify(req.body ?? {}),
      }, opts?.retries ?? 1);

      const text = await response.text();
      res.status(response.status);
      const contentType = response.headers.get("content-type") || "application/json; charset=utf-8";
      res.setHeader("content-type", contentType);
      res.send(text);
    } catch (err) {
      res.status(502).json({
        status: "error",
        code: "PYTHON_EXEC_UNAVAILABLE",
        message: err instanceof Error ? err.message : String(err),
      });
    }
  };

  if (opts?.queue) {
    await enqueue(doRequest);
  } else {
    await doRequest();
  }
}

async function proxyMjpeg(req: Request, res: Response, targetPath: string) {
  try {
    const response = await fetchWithRetry(`${PYTHON_EXEC_BASE}${targetPath}`, { method: "GET" }, 0);
    if (!response.ok || !response.body) {
      res.status(response.status || 502).send("camera stream unavailable");
      return;
    }
    res.status(response.status);
    response.headers.forEach((value, key) => {
      if (key.toLowerCase() === "transfer-encoding") return;
      res.setHeader(key, value);
    });
    const nodeStream = Readable.fromWeb(response.body as any);
    nodeStream.on("error", () => {
      try { res.end(); } catch { /* noop */ }
    });
    nodeStream.pipe(res);
  } catch (err) {
    res.status(502).send(`camera stream unavailable: ${err instanceof Error ? err.message : String(err)}`);
  }
}

app.get("/api/system/health", async (_req, res) => {
  const startedAt = process.uptime();
  try {
    const response = await fetchWithRetry(`${PYTHON_EXEC_BASE}/internal/health`, { method: "GET" }, 0);
    const text = await response.text();
    const base = text ? JSON.parse(text) : {};
    res.status(response.status).json({
      ...base,
      gateway: {
        up: true,
        uptime_s: Math.round(startedAt),
      },
    });
  } catch (err) {
    res.status(200).json({
      status: "degraded",
      gateway: { up: true, uptime_s: Math.round(startedAt) },
      python_exec: { up: false, error: err instanceof Error ? err.message : String(err) },
    });
  }
});

app.get("/api/camera/stream.mjpg", async (req, res) => {
  await proxyMjpeg(req, res, "/internal/camera/stream.mjpg");
});

app.get("/api/config", async (req, res) => proxyJson(req, res, "/internal/config"));
app.put("/api/config", async (req, res) => proxyJson(req, res, "/internal/config", { queue: true }));

app.get("/api/calibration/status", async (req, res) => proxyJson(req, res, "/internal/calibration/status"));
app.post("/api/recovery/search_once", async (req, res) => proxyJson(req, res, "/internal/recovery/search_once", { queue: true }));

const proxiedPostPaths = [
  "/api/fsm/start",
  "/api/fsm/stop",
  "/api/fsm/reset",
  "/api/task/start_cycle",
  "/api/task/pause",
  "/api/task/resume",
  "/api/task/abort",
  "/api/fault/next_pre_suction_fail",
  "/api/fault/drop_once",
  "/api/log/clear",
];

for (const routePath of proxiedPostPaths) {
  app.post(routePath, async (req, res) => {
    const internalPath = routePath.replace(/^\/api\//, "/internal/");
    await proxyJson(req, res, internalPath, { queue: true });
  });
}

app.use("/assets", express.static(path.join(frontendDir, "assets"), { fallthrough: true }));
app.use("/static", express.static(frontendDir, { fallthrough: true }));
app.use(express.static(frontendDir, { extensions: ["html"] }));

app.get("*", (req, res) => {
  if (req.path.startsWith("/api/")) {
    res.status(404).json({ status: "error", message: "API route not found" });
    return;
  }
  res.sendFile(path.join(frontendDir, "index.html"));
});

const server = http.createServer(app);
const wss = new WebSocketServer({ noServer: true });

server.on("upgrade", (request, socket, head) => {
  const url = new URL(request.url || "/", `http://${request.headers.host || "localhost"}`);
  if (url.pathname !== "/ws") {
    socket.destroy();
    return;
  }
  wss.handleUpgrade(request, socket, head, (client) => {
    wss.emit("connection", client, request);
  });
});

wss.on("connection", (client) => {
  let backend: WebSocket | null = null;
  let closed = false;
  let reconnectTimer: NodeJS.Timeout | null = null;
  let attempts = 0;

  const clearReconnect = () => {
    if (reconnectTimer) {
      clearTimeout(reconnectTimer);
      reconnectTimer = null;
    }
  };

  const sendDegraded = (reason: string) => {
    if (client.readyState === WebSocket.OPEN) {
      client.send(JSON.stringify(degradedState(reason)));
    }
  };

  const connectBackend = () => {
    if (closed) return;
    clearReconnect();
    const wsUrl = `${PYTHON_EXEC_WS}/internal/ws`;
    backend = new WebSocket(wsUrl);

    backend.on("open", () => {
      attempts = 0;
    });

    backend.on("message", (data) => {
      if (client.readyState === WebSocket.OPEN) {
        client.send(data.toString());
      }
    });

    backend.on("error", (err) => {
      sendDegraded(`Python execution WS error: ${err.message}`);
    });

    backend.on("close", () => {
      backend = null;
      if (closed) return;
      sendDegraded("Python execution WS disconnected");
      attempts += 1;
      const delay = Math.min(10000, 500 * Math.max(1, attempts));
      reconnectTimer = setTimeout(connectBackend, delay);
    });
  };

  client.on("message", (data) => {
    if (backend && backend.readyState === WebSocket.OPEN) {
      backend.send(data.toString());
    }
  });

  client.on("close", () => {
    closed = true;
    clearReconnect();
    if (backend) backend.close();
  });

  client.on("error", () => {
    closed = true;
    clearReconnect();
    if (backend) backend.close();
  });

  connectBackend();
});

server.listen(PORT, () => {
  console.log(`Gateway listening on http://0.0.0.0:${PORT}`);
  console.log(`Proxying execution service at ${PYTHON_EXEC_BASE}`);
});

