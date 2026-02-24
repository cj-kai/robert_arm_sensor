"""FastAPI execution service: REST + WebSocket + static frontend (legacy direct mode)."""
from __future__ import annotations

import asyncio
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import AsyncIterator

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

from .config_store import ConfigStore
from .fsm import GraspFSM
from .models import LogLevel
from .ws_manager import ws_manager


fsm = GraspFSM()
config_store = ConfigStore()


@asynccontextmanager
async def lifespan(app: FastAPI):
    broadcast_task = asyncio.create_task(broadcast_state())
    print("Execution service started")
    try:
        yield
    finally:
        broadcast_task.cancel()
        try:
            await broadcast_task
        except asyncio.CancelledError:
            pass
        print("Execution service stopped")


app = FastAPI(
    title="Vacuum Grasp Execution Service",
    description="FSM + simulation/hardware adapters for CRX phase-1 project",
    version="1.1.0",
    lifespan=lifespan,
)


async def broadcast_state():
    while True:
        try:
            await ws_manager.broadcast(fsm.state.to_dict())
            await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            break
        except Exception as exc:
            print(f"Broadcast error: {exc}")
            await asyncio.sleep(0.1)


@app.websocket("/ws")
@app.websocket("/internal/ws")
async def websocket_endpoint(websocket: WebSocket):
    await ws_manager.connect(websocket)
    try:
        while True:
            _ = await websocket.receive_text()
    except WebSocketDisconnect:
        await ws_manager.disconnect(websocket)
    except Exception as exc:
        print(f"WebSocket error: {exc}")
        await ws_manager.disconnect(websocket)


async def _ok(message: str, **extra):
    payload = {"status": "ok", "message": message}
    payload.update(extra)
    return payload


@app.post("/api/fsm/start")
@app.post("/internal/fsm/start")
async def api_fsm_start():
    await fsm.start()
    return await _ok("FSM started")


@app.post("/api/fsm/stop")
@app.post("/internal/fsm/stop")
async def api_fsm_stop():
    await fsm.stop()
    return await _ok("FSM stopped")


@app.post("/api/fsm/reset")
@app.post("/internal/fsm/reset")
async def api_fsm_reset():
    await fsm.reset()
    return await _ok("FSM reset")


@app.post("/api/task/start_cycle")
@app.post("/internal/task/start_cycle")
async def api_task_start_cycle():
    await fsm.start_cycle()
    return await _ok("Task cycle started", task=fsm.state.task)


@app.post("/api/task/pause")
@app.post("/internal/task/pause")
async def api_task_pause():
    await fsm.pause_task()
    return await _ok("Task paused", task=fsm.state.task)


@app.post("/api/task/resume")
@app.post("/internal/task/resume")
async def api_task_resume():
    await fsm.resume_task()
    return await _ok("Task resumed", task=fsm.state.task)


@app.post("/api/task/abort")
@app.post("/internal/task/abort")
async def api_task_abort():
    await fsm.abort_task()
    return await _ok("Task aborted", task=fsm.state.task)


@app.post("/api/fault/next_pre_suction_fail")
@app.post("/internal/fault/next_pre_suction_fail")
async def api_fault_pre_suction_fail():
    fsm.inject_pre_suction_fail()
    return await _ok("Fault injected: next pre-suction check will fail")


@app.post("/api/fault/drop_once")
@app.post("/internal/fault/drop_once")
async def api_fault_drop_once():
    fsm.inject_drop_once()
    return await _ok("Fault injected: drop once during transport")


@app.post("/api/recovery/search_once")
@app.post("/internal/recovery/search_once")
async def api_recovery_search_once():
    await fsm.recovery_search_once()
    return await _ok("Manual recovery search triggered")


@app.post("/api/log/clear")
@app.post("/internal/log/clear")
async def api_log_clear():
    fsm.clear_logs()
    return await _ok("Logs cleared")


@app.get("/api/config")
@app.get("/internal/config")
async def api_get_config():
    return config_store.get()


@app.put("/api/config")
@app.put("/internal/config")
async def api_put_config(payload: dict):
    config = config_store.update(payload)
    fsm.state.add_log(LogLevel.INFO, "CONFIG_UPDATED", "Runtime config updated")
    return {"status": "ok", "config": config}


@app.get("/api/system/health")
@app.get("/internal/health")
@app.get("/health")
async def health_check():
    health = fsm.get_health()
    health["ws_connections"] = ws_manager.connection_count
    return health


@app.get("/api/calibration/status")
@app.get("/internal/calibration/status")
async def api_calibration_status():
    return fsm.get_calibration_status()


async def _mjpeg_frame_stream() -> AsyncIterator[bytes]:
    boundary = b"--frame\r\n"
    while True:
        jpg = fsm.get_latest_camera_frame_jpeg()
        headers = (
            boundary
            + b"Content-Type: image/jpeg\r\n"
            + f"Content-Length: {len(jpg)}\r\n\r\n".encode("ascii")
        )
        yield headers + jpg + b"\r\n"
        await asyncio.sleep(0.25)


@app.get("/api/camera/stream.mjpg")
@app.get("/internal/camera/stream.mjpg")
async def api_camera_stream():
    return StreamingResponse(
        _mjpeg_frame_stream(),
        media_type="multipart/x-mixed-replace; boundary=frame",
        headers={"Cache-Control": "no-store"},
    )


STATIC_DIR = Path(__file__).parent.parent / "static"
if STATIC_DIR.exists():
    app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")


@app.get("/")
async def serve_index():
    index_path = STATIC_DIR / "index.html"
    if index_path.exists():
        return FileResponse(index_path, media_type="text/html")
    return {"error": "index.html not found"}
