"""WebSocket 连接管理"""
import asyncio
import json
from typing import List
from fastapi import WebSocket


class WebSocketManager:
    """WebSocket 连接管理器"""

    def __init__(self):
        self._connections: List[WebSocket] = []
        self._lock = asyncio.Lock()

    async def connect(self, websocket: WebSocket):
        """接受新连接"""
        await websocket.accept()
        async with self._lock:
            self._connections.append(websocket)
        print(f"WebSocket connected. Total connections: {len(self._connections)}")

    async def disconnect(self, websocket: WebSocket):
        """断开连接"""
        async with self._lock:
            if websocket in self._connections:
                self._connections.remove(websocket)
        print(f"WebSocket disconnected. Total connections: {len(self._connections)}")

    async def broadcast(self, data: dict):
        """广播消息到所有连接"""
        if not self._connections:
            return

        message = json.dumps(data, ensure_ascii=False)

        # 收集需要移除的断开连接
        disconnected = []

        async with self._lock:
            for connection in self._connections:
                try:
                    await connection.send_text(message)
                except Exception as e:
                    print(f"Failed to send message: {e}")
                    disconnected.append(connection)

            # 移除断开的连接
            for conn in disconnected:
                if conn in self._connections:
                    self._connections.remove(conn)

    @property
    def connection_count(self) -> int:
        """当前连接数"""
        return len(self._connections)


# 全局 WebSocket 管理器实例
ws_manager = WebSocketManager()
