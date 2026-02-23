"""FastAPI 入口：REST API + WebSocket + 静态文件托管"""
import asyncio
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse

from .fsm import GraspFSM
from .ws_manager import ws_manager


# 全局 FSM 实例
fsm = GraspFSM()


@asynccontextmanager
async def lifespan(app: FastAPI):
    """应用生命周期管理"""
    # 启动时：开始 WebSocket 广播任务
    broadcast_task = asyncio.create_task(broadcast_state())
    print("Vacuum Grasp Dashboard started")

    yield

    # 关闭时：取消任务
    broadcast_task.cancel()
    try:
        await broadcast_task
    except asyncio.CancelledError:
        pass
    print("Vacuum Grasp Dashboard stopped")


app = FastAPI(
    title="Vacuum Grasp Dashboard",
    description="真空抓取可靠性与自恢复模块 - 控制台",
    version="1.0.0",
    lifespan=lifespan
)


# ==================== WebSocket ====================

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket 端点：每100ms推送系统状态"""
    await ws_manager.connect(websocket)
    try:
        while True:
            # 保持连接，等待客户端消息（用于心跳）
            data = await websocket.receive_text()
            # 可以处理客户端发来的消息
    except WebSocketDisconnect:
        await ws_manager.disconnect(websocket)
    except Exception as e:
        print(f"WebSocket error: {e}")
        await ws_manager.disconnect(websocket)


async def broadcast_state():
    """后台任务：每50ms广播系统状态"""
    while True:
        try:
            state_data = fsm.state.to_dict()
            await ws_manager.broadcast(state_data)
            await asyncio.sleep(0.05)  # 50ms
        except asyncio.CancelledError:
            break
        except Exception as e:
            print(f"Broadcast error: {e}")
            await asyncio.sleep(0.05)


# ==================== REST API ====================

@app.post("/api/fsm/start")
async def api_fsm_start():
    """启动 FSM"""
    await fsm.start()
    return {"status": "ok", "message": "FSM started"}


@app.post("/api/fsm/stop")
async def api_fsm_stop():
    """停止 FSM"""
    await fsm.stop()
    return {"status": "ok", "message": "FSM stopped"}


@app.post("/api/fsm/reset")
async def api_fsm_reset():
    """重置 FSM"""
    await fsm.reset()
    return {"status": "ok", "message": "FSM reset"}


@app.post("/api/fault/next_pre_suction_fail")
async def api_fault_pre_suction_fail():
    """注入故障：下一次预吸取检查失败"""
    fsm.inject_pre_suction_fail()
    return {"status": "ok", "message": "Fault injected: next pre-suction check will fail"}


@app.post("/api/fault/drop_once")
async def api_fault_drop_once():
    """注入故障：搬运过程中掉压"""
    fsm.inject_drop_once()
    return {"status": "ok", "message": "Fault injected: will drop during transport"}


@app.post("/api/log/clear")
async def api_log_clear():
    """清空日志"""
    fsm.clear_logs()
    return {"status": "ok", "message": "Logs cleared"}


# ==================== 静态文件托管 ====================

# 静态文件目录（Docker 构建时从 frontend 复制到 /app/static）
STATIC_DIR = Path(__file__).parent.parent / "static"

# 挂载静态资源（如果有 js/css 文件）
if STATIC_DIR.exists():
    app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")


@app.get("/")
async def serve_index():
    """提供 index.html"""
    index_path = STATIC_DIR / "index.html"
    if index_path.exists():
        return FileResponse(index_path, media_type="text/html")
    return {"error": "index.html not found"}


# 健康检查
@app.get("/health")
async def health_check():
    """健康检查"""
    return {"status": "healthy"}
