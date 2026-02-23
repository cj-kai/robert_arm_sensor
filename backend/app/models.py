"""状态码、事件码枚举与数据模型"""
from enum import Enum
from dataclasses import dataclass, field
from typing import Optional, List
from collections import deque
import time


class StateCode(str, Enum):
    """FSM 状态码（14个）"""
    IDLE = "IDLE"
    DETECTING_TARGET = "DETECTING_TARGET"
    PLANNING_APPROACH = "PLANNING_APPROACH"
    MOVING_TO_PREGRASP = "MOVING_TO_PREGRASP"
    DESCENDING_TO_CONTACT = "DESCENDING_TO_CONTACT"
    PRE_SUCTION_CHECK = "PRE_SUCTION_CHECK"
    LIFT_VERIFICATION = "LIFT_VERIFICATION"
    TRANSPORT_MONITORING = "TRANSPORT_MONITORING"
    MOVING_TO_PLACE = "MOVING_TO_PLACE"
    RELEASING_LOAD = "RELEASING_LOAD"
    RETURNING_HOME = "RETURNING_HOME"
    AUTO_RETRYING_GRASP = "AUTO_RETRYING_GRASP"
    AUTO_RECOVERY_MODE = "AUTO_RECOVERY_MODE"
    FAULT_LATCHED = "FAULT_LATCHED"


class EventCode(str, Enum):
    """事件码"""
    TARGET_DETECTED = "TARGET_DETECTED"
    APPROACH_PLANNED = "APPROACH_PLANNED"
    VACUUM_ON = "VACUUM_ON"
    VACUUM_OFF_RELEASE = "VACUUM_OFF_RELEASE"
    PRE_SUCTION_OK = "PRE_SUCTION_OK"
    PRE_SUCTION_FAIL = "PRE_SUCTION_FAIL"
    LIFT_CHECK_OK = "LIFT_CHECK_OK"
    LIFT_CHECK_FAIL = "LIFT_CHECK_FAIL"
    DROP_DETECTED = "DROP_DETECTED"
    RECOVERY_TRIGGERED = "RECOVERY_TRIGGERED"
    RETRY_EXHAUSTED = "RETRY_EXHAUSTED"
    CYCLE_SUCCESS = "CYCLE_SUCCESS"


class LogLevel(str, Enum):
    """日志级别"""
    INFO = "INFO"
    WARN = "WARN"
    ERROR = "ERROR"


@dataclass
class LogEntry:
    """日志条目"""
    ts: float
    level: LogLevel
    code: str
    msg: str

    def to_dict(self) -> dict:
        return {
            "ts": self.ts,
            "level": self.level.value,
            "code": self.code,
            "msg": self.msg
        }


def _default_robot_pose() -> dict:
    """默认机器人位姿"""
    return {"x_m": 0.0, "y_m": 0.0, "z_m": 0.3, "roll_deg": 0.0, "pitch_deg": 0.0, "yaw_deg": 0.0}

def _default_target_pose() -> dict:
    """默认目标位姿"""
    return {"x_m": 0.35, "y_m": 0.12, "z_m": 0.0}

def _default_place_pose() -> dict:
    """默认放置位姿"""
    return {"x_m": -0.25, "y_m": 0.25, "z_m": 0.0}

def _default_vision() -> dict:
    """默认视觉状态"""
    return {"detected": False, "confidence": 0.0}

def _default_grip() -> dict:
    """默认夹具状态"""
    return {"vacuum_on": False, "sealed": False}

def _default_fault() -> dict:
    """默认故障状态"""
    return {"active": False, "code": "", "msg": ""}


@dataclass
class SystemState:
    """系统状态"""
    state: StateCode = StateCode.IDLE
    vacuum_ok: bool = False
    vacuum_kpa: Optional[float] = None
    retry_count: int = 0
    recover_count: int = 0
    success_count: int = 0
    last_event: str = ""
    logs: deque = field(default_factory=lambda: deque(maxlen=200))

    # 新增字段（3D可视化）
    robot_pose: dict = field(default_factory=_default_robot_pose)
    target_pose: dict = field(default_factory=_default_target_pose)
    place_pose: dict = field(default_factory=_default_place_pose)
    vision: dict = field(default_factory=_default_vision)
    grip: dict = field(default_factory=_default_grip)
    fault: dict = field(default_factory=_default_fault)

    def add_log(self, level: LogLevel, code: str, msg: str):
        """添加日志"""
        entry = LogEntry(
            ts=time.time(),
            level=level,
            code=code,
            msg=msg
        )
        self.logs.append(entry)
        self.last_event = code

    def clear_logs(self):
        """清空日志"""
        self.logs.clear()

    def get_recent_logs(self, n: int = 50) -> List[dict]:
        """获取最近N条日志"""
        recent = list(self.logs)[-n:]
        return [log.to_dict() for log in recent]

    def to_dict(self) -> dict:
        """转换为字典（用于WebSocket推送）"""
        return {
            "ts": time.time(),
            "state": self.state.value,
            "vacuum_ok": self.vacuum_ok,
            "vacuum_kpa": self.vacuum_kpa,
            "retry_count": self.retry_count,
            "recover_count": self.recover_count,
            "success_count": self.success_count,
            "last_event": self.last_event,
            "log": self.get_recent_logs(50),
            # 新增字段
            "robot_pose": self.robot_pose,
            "target_pose": self.target_pose,
            "place_pose": self.place_pose,
            "vision": self.vision,
            "grip": self.grip,
            "fault": self.fault,
        }


# 3x3 网格偏移顺序（中心→四邻→四角），单位：米
GRID_OFFSETS = [
    (0.0, 0.0),       # 中心
    (0.01, 0.0),      # 右
    (-0.01, 0.0),     # 左
    (0.0, 0.01),      # 前
    (0.0, -0.01),     # 后
    (0.01, 0.01),     # 右前
    (0.01, -0.01),    # 右后
    (-0.01, 0.01),    # 左前
    (-0.01, -0.01),   # 左后
]

MAX_RETRY = 9
