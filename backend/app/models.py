"""Enums and state models for the execution service."""
from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional


class StateCode(str, Enum):
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
    INFO = "INFO"
    WARN = "WARN"
    ERROR = "ERROR"


@dataclass
class LogEntry:
    ts: float
    level: LogLevel
    code: str
    msg: str

    def to_dict(self) -> dict:
        return {
            "ts": self.ts,
            "level": self.level.value,
            "code": self.code,
            "msg": self.msg,
        }


def _default_robot_pose() -> dict:
    return {"x_m": 0.0, "y_m": 0.0, "z_m": 1.0, "roll_deg": 0.0, "pitch_deg": 0.0, "yaw_deg": 0.0}


def _default_target_pose() -> dict:
    return {"x_m": -1.50, "y_m": 1.20, "z_m": 0.80}


def _default_place_pose() -> dict:
    return {"x_m": 1.20, "y_m": 1.00, "z_m": 0.50}


def _default_vision() -> dict:
    return {
        "detected": False,
        "confidence": 0.0,
        "tag_id": None,
        "source": "wrist_usb",
        "camera_ok": False,
        "pose_valid": False,
    }


def _default_grip() -> dict:
    return {"vacuum_on": False, "sealed": False}


def _default_fault() -> dict:
    return {"active": False, "code": "", "msg": ""}


def _default_joint_angles_rad() -> list:
    return [0.0, -1.1, 1.6, 0.6, 1.57, 0.0]


def _default_conveyor() -> dict:
    return {
        "mode": "real_io",
        "running_cmd": False,
        "running_fb": False,
        "ready": True,
        "last_start_ts": 0.0,
        "timeout_active": False,
    }


def _default_vacuum_meta() -> dict:
    return {
        "mode": "simulated_io",
        "pressure_kpa": None,
        "sensor_ok": False,
    }


def _default_robot_meta() -> dict:
    return {
        "mode": "simulated",
        "connected": False,
        "servo_enabled": False,
        "motion_busy": False,
        "estop_latched": False,
    }


def _default_task() -> dict:
    return {
        "name": "clean_to_conveyor_to_dirty",
        "phase": "WAITING",
        "cycle_id": "",
        "paused": False,
    }


def _default_safety() -> dict:
    return {
        "web_motion_allowed": False,
        "manual_jog_enabled": False,
    }


def _default_calibration() -> dict:
    return {
        "camera_intrinsics": False,
        "hand_eye": False,
        "workcell_points": False,
        "updated_at": 0.0,
    }


def _default_payload() -> dict:
    return {
        "kind": "square_tray_10_75",
        "label": "Square Tray 10.75",
        "dims_m": {"x_m": 0.2731, "y_m": 0.2731, "z_m": 0.0318},
        "tag_size_m": 0.05,
        "is_clean": False,
    }


def _default_washer() -> dict:
    return {
        "model": "MIMASA DA-80",
        "mode": "simulated",
        "busy": False,
        "done": False,
        "cycle_time_s": 12.0,
        "remaining_s": 0.0,
    }


@dataclass
class SystemState:
    state: StateCode = StateCode.IDLE
    vacuum_ok: bool = False
    vacuum_kpa: Optional[float] = None
    retry_count: int = 0
    recover_count: int = 0
    success_count: int = 0
    last_event: str = ""
    logs: deque = field(default_factory=lambda: deque(maxlen=200))

    robot_pose: dict = field(default_factory=_default_robot_pose)
    target_pose: dict = field(default_factory=_default_target_pose)
    place_pose: dict = field(default_factory=_default_place_pose)
    vision: dict = field(default_factory=_default_vision)
    grip: dict = field(default_factory=_default_grip)
    fault: dict = field(default_factory=_default_fault)

    # Phase-1 extended payload for node gateway / hardware-ready interfaces.
    joint_angles_rad: list = field(default_factory=_default_joint_angles_rad)
    conveyor: dict = field(default_factory=_default_conveyor)
    vacuum: dict = field(default_factory=_default_vacuum_meta)
    robot: dict = field(default_factory=_default_robot_meta)
    task: dict = field(default_factory=_default_task)
    safety: dict = field(default_factory=_default_safety)
    calibration: dict = field(default_factory=_default_calibration)
    payload: dict = field(default_factory=_default_payload)
    washer: dict = field(default_factory=_default_washer)

    def add_log(self, level: LogLevel, code: str, msg: str) -> None:
        entry = LogEntry(ts=time.time(), level=level, code=code, msg=msg)
        self.logs.append(entry)
        self.last_event = code

    def clear_logs(self) -> None:
        self.logs.clear()

    def get_recent_logs(self, n: int = 50) -> List[dict]:
        return [log.to_dict() for log in list(self.logs)[-n:]]

    def to_dict(self) -> dict:
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
            "robot_pose": self.robot_pose,
            "target_pose": self.target_pose,
            "place_pose": self.place_pose,
            "vision": self.vision,
            "grip": self.grip,
            "fault": self.fault,
            "joint_angles_rad": self.joint_angles_rad,
            "conveyor": self.conveyor,
            "vacuum": self.vacuum,
            "robot": self.robot,
            "task": self.task,
            "safety": self.safety,
            "calibration": self.calibration,
            "payload": self.payload,
            "washer": self.washer,
        }


GRID_OFFSETS = [
    (0.0, 0.0),
    (0.01, 0.0),
    (-0.01, 0.0),
    (0.0, 0.01),
    (0.0, -0.01),
    (0.01, 0.01),
    (0.01, -0.01),
    (-0.01, 0.01),
    (-0.01, -0.01),
]

MAX_RETRY = 9
