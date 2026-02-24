"""Hardware adapter interfaces and simulated implementations for phase-1 execution service."""
from __future__ import annotations

import base64
import time
from dataclasses import dataclass
from typing import Optional, Protocol

from .simulator import VisionSim, VacuumSim, RobotSim, GripSim


@dataclass
class Ack:
    ok: bool
    msg: str = ""

    def to_dict(self) -> dict:
        return {"ok": self.ok, "msg": self.msg}


class RobotAdapter(Protocol):
    def connect(self) -> bool: ...
    def is_connected(self) -> bool: ...
    def get_joint_angles_rad(self) -> list[float]: ...
    def get_tcp_pose(self) -> dict: ...
    def move_named_pose(self, name: str) -> Ack: ...
    def move_pose(self, pose: dict, speed_scale: float) -> Ack: ...
    def wait_motion_done(self, timeout_s: float) -> bool: ...
    def stop_motion(self) -> Ack: ...
    def reset_fault(self) -> Ack: ...
    def get_status(self) -> dict: ...


class VisionAdapter(Protocol):
    def start(self) -> None: ...
    def stop(self) -> None: ...
    def get_latest_frame_jpeg(self) -> bytes: ...
    def detect_tray_tag(self) -> dict: ...
    def camera_ok(self) -> bool: ...
    def get_status(self) -> dict: ...


class VacuumAdapter(Protocol):
    mode: str
    def set_vacuum(self, on: bool) -> Ack: ...
    def read_pressure_kpa(self) -> Optional[float]: ...
    def is_sealed(self) -> bool: ...
    def get_status(self) -> dict: ...


class ConveyorAdapter(Protocol):
    def start(self) -> Ack: ...
    def stop(self) -> Ack: ...
    def is_running(self) -> bool: ...
    def is_ready(self) -> bool: ...
    def get_status(self) -> dict: ...


_STATIC_JPEG = base64.b64decode(
    # 1x1 gray jpeg
    b"/9j/4AAQSkZJRgABAQAAAQABAAD/2wCEAAkGBxAQEBAQEBAPEA8QDxAQDw8QEA8QEA8PFREWFhUR"
    b"FRUYHSggGBolGxUVITEhJSkrLi4uFx8zODMsNygtLisBCgoKDg0OGxAQGy0lICUtLS0tLS0tLS0tLS0t"
    b"LS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLf/AABEIAAEAAQMBIgACEQEDEQH/xAAXAAEBAQAA"
    b"AAAAAAAAAAAAAAAAAQID/8QAFhABAQEAAAAAAAAAAAAAAAAAAAER/9oADAMBAAIQAxAAAAHLiP/EABgQ"
    b"AQEBAQEAAAAAAAAAAAAAAAERACEx/9oACAEBAAEFAot6e0//xAAVEQEBAAAAAAAAAAAAAAAAAAABEP/a"
    b"AAgBAwEBPwGn/8QAFBEBAAAAAAAAAAAAAAAAAAAAEP/aAAgBAgEBPwEf/8QAGRAAAwADAAAAAAAAAAAA"
    b"AAAAAAERITFB/9oACAEBAAY/AvFhaqf/xAAaEAACAwEBAAAAAAAAAAAAAAABEQAhMUFR/9oACAEBAAE/"
    b"IcyRLCkN2S9Y8uP/2gAMAwEAAgADAAAAEB//xAAVEQEBAAAAAAAAAAAAAAAAAAAAEf/aAAgBAwEBPxA8"
    b"/8QAFBEBAAAAAAAAAAAAAAAAAAAAEP/aAAgBAgEBPxAf/8QAGhABAAMBAQEAAAAAAAAAAAAAAQARITFB"
    b"cf/aAAgBAQABPxCv0YkQ1aQqeN2nJ5L/2Q=="
)


class SimRobotAdapter:
    def __init__(self, robot: RobotSim):
        self.robot = robot
        self._connected = True
        self._servo_enabled = True
        self._motion_busy_until = 0.0

    def connect(self) -> bool:
        self._connected = True
        return True

    def is_connected(self) -> bool:
        return self._connected

    def get_joint_angles_rad(self) -> list[float]:
        return self.robot.get_joint_angles_rad()

    def get_tcp_pose(self) -> dict:
        return self.robot.get_pose_dict()

    def move_named_pose(self, name: str) -> Ack:
        self._motion_busy_until = time.time() + 0.5
        return Ack(True, f"Sim move to named pose: {name}")

    def move_pose(self, pose: dict, speed_scale: float) -> Ack:
        self._motion_busy_until = time.time() + 0.5
        return Ack(True, f"Sim move pose accepted (speed_scale={speed_scale})")

    def wait_motion_done(self, timeout_s: float) -> bool:
        end = time.time() + max(0.0, timeout_s)
        while time.time() < end:
            if time.time() >= self._motion_busy_until:
                return True
            time.sleep(0.01)
        return time.time() >= self._motion_busy_until

    def stop_motion(self) -> Ack:
        self._motion_busy_until = time.time()
        self.robot.stop()
        return Ack(True, "Sim motion stopped")

    def reset_fault(self) -> Ack:
        return Ack(True, "No robot faults in sim")

    def get_status(self) -> dict:
        return {
            "mode": "simulated",
            "connected": self._connected,
            "servo_enabled": self._servo_enabled,
            "motion_busy": time.time() < self._motion_busy_until,
            "estop_latched": False,
        }


class SimVisionAdapter:
    def __init__(self, vision: VisionSim):
        self.vision = vision
        self._running = True

    def start(self) -> None:
        self._running = True
        self.vision.start_scan()

    def stop(self) -> None:
        self._running = False

    def get_latest_frame_jpeg(self) -> bytes:
        return _STATIC_JPEG

    def detect_tray_tag(self) -> dict:
        d = self.vision.detect()
        return {
            "detected": d.get("detected", False),
            "confidence": d.get("confidence", 0.0),
            "tag_id": 1 if d.get("detected") else None,
            "pose_valid": bool(d.get("detected")),
            "source": "wrist_usb",
            "camera_ok": self.camera_ok(),
            "target_pose": d.get("target_pose"),
        }

    def camera_ok(self) -> bool:
        return self._running

    def get_status(self) -> dict:
        return {
            "mode": "simulated_tag",
            "camera_ok": self.camera_ok(),
            "source": "wrist_usb",
        }


class SimVacuumAdapter:
    mode = "simulated_io"

    def __init__(self, vacuum: VacuumSim, grip: GripSim):
        self.vacuum = vacuum
        self.grip = grip

    def set_vacuum(self, on: bool) -> Ack:
        if on:
            self.vacuum.turn_on()
        else:
            self.vacuum.turn_off()
        return Ack(True, f"Vacuum {'ON' if on else 'OFF'}")

    def read_pressure_kpa(self) -> Optional[float]:
        if not self.vacuum.vacuum_on:
            return None
        return self.vacuum.read_pressure()

    def is_sealed(self) -> bool:
        return bool(self.grip.sealed)

    def get_status(self) -> dict:
        return {
            "mode": self.mode,
            "pressure_kpa": self.vacuum.vacuum_kpa if self.vacuum.vacuum_on else None,
            "sensor_ok": False,
        }


class SimConveyorAdapter:
    def __init__(self, mode: str = "real_io"):
        self.mode = mode
        self._running_cmd = False
        self._running_fb = False
        self._ready = True
        self._last_start_ts = 0.0
        self._timeout_active = False

    def start(self) -> Ack:
        self._running_cmd = True
        self._running_fb = True
        self._last_start_ts = time.time()
        self._timeout_active = False
        return Ack(True, "Conveyor started (sim)")

    def stop(self) -> Ack:
        self._running_cmd = False
        self._running_fb = False
        self._timeout_active = False
        return Ack(True, "Conveyor stopped (sim)")

    def is_running(self) -> bool:
        return self._running_fb

    def is_ready(self) -> bool:
        return self._ready

    def get_status(self) -> dict:
        return {
            "mode": self.mode,
            "running_cmd": self._running_cmd,
            "running_fb": self._running_fb,
            "ready": self._ready,
            "last_start_ts": self._last_start_ts,
            "timeout_active": self._timeout_active,
        }

