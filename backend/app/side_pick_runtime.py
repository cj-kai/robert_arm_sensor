"""Execution-service compatible wrapper around VisionGuidedSidePickFSM.

This adapter keeps the current FastAPI routes and websocket payload shape mostly
unchanged while driving the newer, more realistic side-pick FSM.
"""
from __future__ import annotations

import time
import uuid
from typing import Optional

from .adapters import SimConveyorAdapter, SimRobotAdapter, SimVisionAdapter
from .config_store import DEFAULT_CONFIG
from .models import LogLevel, StateCode, SystemState
from .vision_side_pick_fsm import SidePickState, VisionGuidedSidePickFSM


class SidePickExecutionFSM:
    """Drop-in-ish replacement for `GraspFSM` used by `main.py`."""

    def __init__(self):
        self.core = VisionGuidedSidePickFSM()
        self.state = SystemState()

        self._running = False
        self._paused = False
        self._sim_speed = 1.0
        self._sim_paused = False
        self._cycle_name = "clean_to_conveyor_to_dirty"
        self._cycle_id = ""
        self._task_phase_override: Optional[str] = "WAITING"
        self._last_update_ts = time.time()
        self._last_core_state = self.core.state
        self._last_seg_name = ""

        self._vision_adapter = SimVisionAdapter(self.core.vision)
        self._robot_adapter = SimRobotAdapter(self.core.robot)
        self._conveyor = SimConveyorAdapter(mode="real_io")
        self._calibration_status = {
            "camera_intrinsics": False,
            "hand_eye": False,
            "workcell_points": False,
            "updated_at": 0.0,
        }
        self._tray_profiles = list(DEFAULT_CONFIG.get("tray_profiles", {}).values())
        self._sync_state_from_core()

    # ---- main loop hook (called from broadcast task) ----

    def update(self, dt: float) -> None:
        dt = max(0.0, min(0.1, float(dt)))
        if self._sim_paused:
            dt = 0.0
        else:
            dt *= max(0.1, min(4.0, self._sim_speed))
        if self._running and not self._paused:
            self.core.step(dt)
        self._sync_state_from_core()

    # ---- API-compatible control methods ----

    async def start(self):
        if self._running:
            return
        self._running = True
        self._paused = False
        self._task_phase_override = None
        self._vision_adapter.start()
        self.state.add_log(LogLevel.INFO, "FSM_STARTED", "Side-pick FSM started")
        self._sync_state_from_core()

    async def start_cycle(self, task_name: str = "clean_to_conveyor_to_dirty"):
        self._cycle_name = task_name or self._cycle_name
        self._cycle_id = uuid.uuid4().hex[:10]
        # If previous cycle finished, keep current clean stack count but reset dirty source when empty.
        if self.core.state == SidePickState.FAULT:
            self.core = VisionGuidedSidePickFSM()
        self.state.task.update({
            "name": self._cycle_name,
            "phase": "STARTING",
            "cycle_id": self._cycle_id,
            "paused": False,
        })
        await self.start()

    async def pause_task(self):
        if not self._running:
            return
        self._paused = True
        self.state.task["paused"] = True
        self.state.add_log(LogLevel.WARN, "TASK_PAUSED", "Task paused by operator")
        self._sync_state_from_core()

    async def resume_task(self):
        if not self._running:
            return
        self._paused = False
        self.state.task["paused"] = False
        self.state.add_log(LogLevel.INFO, "TASK_RESUMED", "Task resumed by operator")
        self._sync_state_from_core()

    async def abort_task(self):
        self.state.add_log(LogLevel.WARN, "TASK_ABORT", "Task abort requested")
        await self.stop()

    async def stop(self):
        self._running = False
        self._paused = False
        self._task_phase_override = "STOPPED"
        self._vision_adapter.stop()
        self._conveyor.stop()
        self.core.vacuum.turn_off()
        self.state.task["paused"] = False
        self.state.add_log(LogLevel.INFO, "FSM_STOPPED", "FSM stopped, returned to IDLE")
        self._sync_state_from_core(force_idle=True)

    async def reset(self):
        self.core = VisionGuidedSidePickFSM()
        self._vision_adapter = SimVisionAdapter(self.core.vision)
        self._robot_adapter = SimRobotAdapter(self.core.robot)
        self._running = False
        self._paused = False
        self._cycle_id = ""
        self._task_phase_override = "WAITING"

        self.state = SystemState()
        self.state.add_log(LogLevel.INFO, "FSM_RESET", "FSM reset to initial state")
        self._sync_state_from_core(force_idle=True)

    async def recovery_search_once(self):
        self.state.recover_count += 1
        self._task_phase_override = "RECOVERY_SEARCH"
        self.state.add_log(LogLevel.WARN, "MANUAL_RECOVERY_SEARCH", "Manual recovery search triggered")
        self._sync_state_from_core()

    def inject_pre_suction_fail(self):
        self.core.vacuum.inject_pre_suction_fail()
        self.state.add_log(LogLevel.WARN, "FAULT_INJECT", "Injected: next pre-suction check will fail")

    def inject_drop_once(self):
        # The new side-pick FSM does not yet consume drop fault injection; keep API compatible.
        self.core.vacuum.inject_drop_once()
        self.state.add_log(LogLevel.WARN, "FAULT_INJECT", "Injected: drop once requested (not yet modeled in side-pick FSM)")

    def clear_logs(self):
        self.state.clear_logs()
        self.state.add_log(LogLevel.INFO, "LOG_CLEARED", "Log buffer cleared")

    # ---- playback / param control ----

    def set_sim_speed(self, speed: float) -> None:
        self._sim_speed = max(0.1, min(4.0, float(speed)))
        self.state.add_log(LogLevel.INFO, "SIM_SPEED", f"Speed set to {self._sim_speed:.1f}x")

    def set_sim_paused(self, paused: bool) -> None:
        self._sim_paused = bool(paused)
        action = "paused" if self._sim_paused else "resumed"
        self.state.add_log(LogLevel.INFO, "SIM_PLAYBACK", f"Simulation {action}")

    def update_params(self, params: dict) -> None:
        cfg = self.core.cfg
        if "max_joint_speed" in params:
            self.core.MAX_JOINT_SPEED_RAD_S = max(0.5, min(4.0, float(params["max_joint_speed"])))
        if "approach_standoff" in params:
            cfg.approach_standoff_m = max(0.10, min(0.30, float(params["approach_standoff"])))
        if "vision_noise" in params:
            noise = max(0.0, min(0.02, float(params["vision_noise"])))
            cfg.vision_noise_xy_m = noise
            self.core.sim_vision.noise_std_m = noise
        if "vision_fail_prob" in params:
            prob = max(0.0, min(0.20, float(params["vision_fail_prob"])))
            cfg.vision_fail_prob = prob
            self.core.sim_vision.fail_prob = prob
        if "randomize_tray_size" in params:
            cfg.randomize_tray_size = bool(params["randomize_tray_size"])
        if "randomize_tray_position" in params:
            cfg.randomize_tray_position = bool(params["randomize_tray_position"])
        if "sim_speed" in params:
            self.set_sim_speed(float(params["sim_speed"]))
        if "sim_paused" in params:
            self.set_sim_paused(bool(params["sim_paused"]))
        self.state.add_log(LogLevel.INFO, "PARAM_UPDATE", f"Parameters updated: {list(params.keys())}")

    # ---- health / calibration / camera API ----

    def get_health(self) -> dict:
        return {
            "status": "ok",
            "python_exec": {"up": True},
            "robot": self._robot_adapter.get_status(),
            "camera": self._vision_adapter.get_status(),
            "conveyor": self._conveyor.get_status(),
            "vacuum": self.state.vacuum,
            "fsm": {"running": self._running, "paused": self._paused, "state": self.state.state.value},
        }

    def get_calibration_status(self) -> dict:
        return dict(self._calibration_status)

    def get_latest_camera_frame_jpeg(self) -> bytes:
        return self._vision_adapter.get_latest_frame_jpeg()

    # ---- internal state sync ----

    def _sync_state_from_core(self, force_idle: bool = False) -> None:
        snap = self.core.snapshot()
        now = time.time()
        self._last_update_ts = now

        self._sync_logs()

        core_state = self.core.state
        seg_name = self.core._traj.current.name if self.core._traj and self.core._traj.current else ""
        self.state.state = StateCode.IDLE if force_idle else self._map_state(core_state, seg_name)

        self.state.retry_count = 0
        self.state.recover_count = max(self.state.recover_count, 0)
        self.state.success_count = self.core.total_cycles

        self.state.robot_pose = dict(snap["robot_pose"])
        try:
            self.state.joint_angles_rad = self.core.get_joint_angles_rad()
        except Exception:
            self.state.joint_angles_rad = self.core.robot.get_joint_angles_rad()

        vis = self.core.vision.detect()
        det = snap.get("detection", {})
        self.state.vision = {
            "detected": bool(det.get("detected", vis.get("detected", False))),
            "confidence": round(float(det.get("confidence", vis.get("confidence", 0.0))), 3),
            "tag_id": vis.get("tag_id"),
            "source": "wrist_usb",
            "camera_ok": self._vision_adapter.camera_ok(),
            "pose_valid": bool(det.get("detected", vis.get("pose_valid", False))),
            "detection_bbox": det.get("bbox"),
            "detection_noise_m": det.get("noise_std_m", 0.0),
            "detection_latency_ms": det.get("latency_ms", 0.0),
            "camera_frame_id": det.get("camera_frame_id", 0),
            "tray_size_label": det.get("tray_size_label", ""),
            "detect_retries": det.get("detect_retries", 0),
        }

        target = self.core._current_target
        if target is not None:
            self.state.target_pose = {"x_m": target.x, "y_m": target.y, "z_m": target.z}
        else:
            top = self.core._peek_dirty_tray()
            if top is not None:
                self.state.target_pose = {
                    "x_m": top.pose.pos.x,
                    "y_m": top.pose.pos.y,
                    "z_m": top.pose.pos.z,
                }

        place_pose = self._current_place_pose()
        self.state.place_pose = {"x_m": place_pose[0], "y_m": place_pose[1], "z_m": place_pose[2]}

        tray_for_payload = self._visible_tray()
        self.state.payload = self._payload_dict(tray_for_payload)
        self.state.tray_body = self._tray_body_dict(tray_for_payload)

        vacuum_on = bool(self.core.vacuum.vacuum_on)
        pressure = self.core.vacuum.read_pressure() if vacuum_on else None
        bound = bool(self.core._active_tray and self.core._active_tray.bound_to_tool)
        self.state.vacuum_kpa = pressure
        self.state.vacuum_ok = bool(self.core.vacuum.vacuum_ok) if vacuum_on else False
        self.state.grip = {"vacuum_on": vacuum_on, "sealed": bound}
        self.state.vacuum = {
            "mode": "simulated_io",
            "pressure_kpa": pressure,
            "sensor_ok": False,
        }

        self._sync_conveyor_and_washer()

        self.state.robot = self._robot_adapter.get_status()
        self.state.robot["connected"] = True

        self.state.task = {
            "name": self._cycle_name,
            "phase": self._derive_task_phase(core_state, seg_name, force_idle),
            "cycle_id": self._cycle_id,
            "paused": self._paused,
        }
        self.state.safety = {"web_motion_allowed": False, "manual_jog_enabled": False}
        self.state.calibration = dict(self._calibration_status)

        traj = self.core._traj
        if traj and traj.segments:
            pts = []
            for seg in traj.segments:
                pts.append({"x_m": seg.start.pos.x, "y_m": seg.start.pos.y, "z_m": seg.start.pos.z})
            last = traj.segments[-1]
            pts.append({"x_m": last.end.pos.x, "y_m": last.end.pos.y, "z_m": last.end.pos.z})
            self.state.trajectory_preview = pts
        else:
            self.state.trajectory_preview = []

        self.state.proximity = snap.get("proximity", {"distance_m": None, "triggered": False, "threshold_m": 0.03})
        self.state.force_sensor = snap.get("force_sensor", {"fx": 0, "fy": 0, "fz": 0, "tx": 0, "ty": 0, "tz": 0, "payload_kg": 0})
        self.state.sim_speed = self._sim_speed
        self.state.sim_paused = self._sim_paused

        if self.core.state == SidePickState.FAULT:
            self.state.fault = {"active": True, "code": "SIDE_PICK_FAULT", "msg": self.core.last_error or "Side-pick FSM fault"}
        else:
            self.state.fault = {"active": False, "code": "", "msg": ""}

        if force_idle:
            self.state.state = StateCode.IDLE
            self.state.task["phase"] = self._task_phase_override or "WAITING"

    def _sync_logs(self) -> None:
        core_state = self.core.state
        seg_name = self.core._traj.current.name if self.core._traj and self.core._traj.current else ""

        if core_state != self._last_core_state:
            self.state.add_log(LogLevel.INFO, "STATE_CHANGE", f"{self._last_core_state.value if self._last_core_state else 'NONE'} -> {core_state.value}")
            self._last_core_state = core_state
        if seg_name and seg_name != self._last_seg_name:
            self.state.add_log(LogLevel.INFO, "SEGMENT_START", seg_name)
            self._last_seg_name = seg_name
        if not seg_name:
            self._last_seg_name = ""

    def _map_state(self, core_state: SidePickState, seg_name: str) -> StateCode:
        if core_state == SidePickState.DETECT_DIRTY:
            return StateCode.DETECTING_TARGET
        if core_state in {SidePickState.PICK_DIRTY_SIDE, SidePickState.PICK_CLEAN_SIDE}:
            if seg_name.startswith("pre_grasp"):
                return StateCode.MOVING_TO_PREGRASP
            if "contact" in seg_name:
                return StateCode.DESCENDING_TO_CONTACT
            if "vacuum_on" in seg_name:
                return StateCode.PRE_SUCTION_CHECK
            return StateCode.LIFT_VERIFICATION
        if core_state in {SidePickState.PLACE_TO_WASHER, SidePickState.PLACE_CLEAN}:
            if "release" in seg_name:
                return StateCode.RELEASING_LOAD
            if "insert" in seg_name or "contact" in seg_name:
                return StateCode.MOVING_TO_PLACE
            if "retreat" in seg_name or "home" in seg_name or "clear" in seg_name:
                return StateCode.RETURNING_HOME
            return StateCode.TRANSPORT_MONITORING
        if core_state == SidePickState.WAIT_AND_DETECT_CLEAN:
            return StateCode.RETURNING_HOME
        if core_state == SidePickState.FAULT:
            return StateCode.FAULT_LATCHED
        return StateCode.IDLE

    def _derive_task_phase(self, core_state: SidePickState, seg_name: str, force_idle: bool) -> str:
        if force_idle:
            return self._task_phase_override or "WAITING"
        if self._task_phase_override and self._task_phase_override in {"STOPPED", "RECOVERY_SEARCH"}:
            return self._task_phase_override
        if self._paused:
            return "PAUSED"
        if not self._running and core_state != SidePickState.FAULT:
            return "WAITING"
        if core_state == SidePickState.DETECT_DIRTY:
            return "RACK_PICK"
        if core_state == SidePickState.PICK_DIRTY_SIDE:
            return "RACK_PICK"
        if core_state == SidePickState.PLACE_TO_WASHER:
            return "MOVE_TO_WASHER_LOAD"
        if core_state == SidePickState.WAIT_AND_DETECT_CLEAN:
            return "WASHER_PROCESS"
        if core_state == SidePickState.PICK_CLEAN_SIDE:
            return "RETURN_PICK"
        if core_state == SidePickState.PLACE_CLEAN:
            return "MOVE_TO_CLEAN_RACK"
        if core_state == SidePickState.FAULT:
            return "FAULT"
        return "WAITING"

    def _sync_conveyor_and_washer(self) -> None:
        washer_tray = self.core._washer_tray
        busy = bool(washer_tray and not washer_tray.is_clean)
        done = bool(washer_tray and washer_tray.is_clean)
        if busy:
            self._conveyor._running_cmd = True
            self._conveyor._running_fb = True
        else:
            self._conveyor._running_cmd = False
            self._conveyor._running_fb = False

        remaining = 0.0
        if busy:
            remaining = max(0.0, self.core.cfg.washer_cycle_s - self.core._washer_elapsed_s)
        flow_stage = (
            "DIRTY_TO_WASHER" if self.core.state in {SidePickState.DETECT_DIRTY, SidePickState.PICK_DIRTY_SIDE, SidePickState.PLACE_TO_WASHER}
            else "WASHER_PROCESS" if self.core.state == SidePickState.WAIT_AND_DETECT_CLEAN
            else "WASHER_TO_CLEAN"
        )

        self.state.conveyor = self._conveyor.get_status()
        self.state.washer = {
            "model": "MIMASA DA-80",
            "mode": "simulated",
            "busy": busy,
            "done": done,
            "cycle_time_s": self.core.cfg.washer_cycle_s,
            "remaining_s": round(remaining, 1),
            "flow_stage": flow_stage,
        }

    def _visible_tray(self):
        if self.core._active_tray is not None:
            return self.core._active_tray
        if self.core._washer_tray is not None:
            return self.core._washer_tray
        if getattr(self.core, "_last_placed_clean_tray", None) is not None:
            return self.core._last_placed_clean_tray
        return self.core._peek_dirty_tray()

    def _payload_dict(self, tray) -> dict:
        if tray is None:
            profile = self._tray_profiles[0] if self._tray_profiles else {"label": "Square Tray 10.75", "dims_m": {"x_m": 0.4, "y_m": 0.4, "z_m": 0.05}, "tag_size_m": 0.05}
            return {
                "kind": "square_tray_10_75",
                "label": profile.get("label", "Tray"),
                "dims_m": dict(profile.get("dims_m", {"x_m": 0.4, "y_m": 0.4, "z_m": 0.05})),
                "tag_size_m": float(profile.get("tag_size_m", 0.05)),
                "is_clean": False,
            }
        return {
            "kind": "tray",
            "label": "Clean Tray" if tray.is_clean else "Dirty Tray",
            "dims_m": {"x_m": tray.dims_m.x, "y_m": tray.dims_m.y, "z_m": tray.dims_m.z},
            "tag_size_m": 0.05,
            "is_clean": bool(tray.is_clean),
        }

    def _tray_body_dict(self, tray) -> dict:
        if tray is None:
            return {"visible": False}
        return {
            "visible": True,
            "tray_id": tray.tray_id,
            "bound_to_tool": bool(tray.bound_to_tool),
            "gravity_enabled": bool(tray.gravity_enabled),
            "is_clean": bool(tray.is_clean),
            "dims_m": {"x_m": tray.dims_m.x, "y_m": tray.dims_m.y, "z_m": tray.dims_m.z},
            "pose": {
                "x_m": tray.pose.pos.x,
                "y_m": tray.pose.pos.y,
                "z_m": tray.pose.pos.z,
            },
        }

    def _current_place_pose(self) -> tuple[float, float, float]:
        if self.core.state in {SidePickState.PLACE_CLEAN, SidePickState.PICK_CLEAN_SIDE}:
            z = self.core.layout.clean_rack_place_base.z + self.core.clean_stack_count * self.core.cfg.tray_thickness_m
            return (self.core.layout.clean_rack_place_base.x, self.core.layout.clean_rack_place_base.y, z)
        return (self.core.layout.washer_infeed.x, self.core.layout.washer_infeed.y, self.core.layout.washer_infeed.z)
