"""doc"""
import asyncio
import time
import uuid
from typing import Optional
from .models import (
    StateCode, EventCode, LogLevel, SystemState,
    GRID_OFFSETS, MAX_RETRY
)
from .simulator import VisionSim, VacuumSim, RobotSim, GripSim, TargetPose
from .adapters import SimRobotAdapter, SimVisionAdapter, SimVacuumAdapter, SimConveyorAdapter
from .config_store import DEFAULT_CONFIG


class GraspFSM:
    """doc"""

    def __init__(self):
        self.state = SystemState()
        self.vision = VisionSim()
        self.vacuum = VacuumSim()
        self.robot = RobotSim()
        self.grip = GripSim()
        self.conveyor = SimConveyorAdapter(mode="real_io")
        self.robot_adapter = SimRobotAdapter(self.robot)
        self.vision_adapter = SimVisionAdapter(self.vision)
        self.vacuum_adapter = SimVacuumAdapter(self.vacuum, self.grip)

        # 杩愯鎺у埗
        self._running: bool = False
        self._task: Optional[asyncio.Task] = None
        self._paused: bool = False

        # 褰撳墠鐩爣浣嶇疆
        self._current_target: Optional[TargetPose] = None
        self._current_offset_idx: int = 0

        # 鐩爣浣嶅Э锛堢敤浜庡钩婊戠Щ鍔級
        self._goal_pose: Optional[dict] = None

        # 鏀剧疆浣嶇疆锛堜笘鐣屽潗鏍囷紝鍗曚綅锛氱背锟?
        self._place_position = {"x_m": 1.20, "y_m": 1.00, "z_m": 0.50}

        # 瀹夊叏浣嶅Э
        self._safe_pose = {"x_m": 0.0, "y_m": 0.0, "z_m": 1.0}

        # 鎺ヨЕ楂樺害
        self._contact_z = 0.50  # default mid-height contact in phase-1 tray washer layout

        # 鐘舵€佽鏃跺櫒
        self._state_timer: float = 0.0

        # 鏁呴殰鏍囧織
        self._fault_injected: bool = False
        self._cycle_name: str = "clean_to_conveyor_to_dirty"
        self._cycle_id: str = ""
        self._task_phase_override: Optional[str] = "WAITING"
        self._calibration_status = {
            "camera_intrinsics": False,
            "hand_eye": False,
            "workcell_points": False,
            "updated_at": 0.0,
        }
        self._tray_profiles = list(DEFAULT_CONFIG.get("tray_profiles", {}).items())
        self._tray_profile_idx = 0
        self._active_payload = self._make_payload(self._tray_profiles[0][0], self._tray_profiles[0][1]) if self._tray_profiles else {
            "kind": "square_tray_10_75",
            "label": "Square Tray 10.75",
            "dims_m": {"x_m": 0.2731, "y_m": 0.2731, "z_m": 0.0318},
            "tag_size_m": 0.05,
            "is_clean": False,
        }
        stations = DEFAULT_CONFIG.get("stations", {})
        self._dirty_pick = dict(stations.get("rack_dirty_pick", {"x_m": 0.35, "y_m": 0.18, "z_m": 0.0}))
        self._clean_place = dict(stations.get("rack_clean_place", {"x_m": 0.35, "y_m": -0.18, "z_m": 0.0}))
        self._washer_load = dict(stations.get("washer_load", {"x_m": -0.15, "y_m": 0.15, "z_m": 0.05}))
        self._washer_return_pick = dict(stations.get("washer_return_pick", {"x_m": -0.18, "y_m": -0.10, "z_m": 0.05}))
        self._flow_stage = "DIRTY_TO_WASHER"
        self._washer_cycle_s = float(DEFAULT_CONFIG.get("washer", {}).get("cycle_time_s", 12.0))
        self._washer_started_ts = 0.0
        self._washer_busy = False
        self._washer_done = False
        self.robot.safe_z = self._safe_pose["z_m"]
        self._prepare_cycle_targets(reset_payload=False)

    def _make_payload(self, key: str, spec: dict) -> dict:
        return {
            "kind": key,
            "label": spec.get("label", key),
            "dims_m": dict(spec.get("dims_m", {"x_m": 0.25, "y_m": 0.20, "z_m": 0.03})),
            "tag_size_m": float(spec.get("tag_size_m", 0.05)),
            "is_clean": False,
        }

    def _next_payload(self) -> dict:
        if not self._tray_profiles:
            return dict(self._active_payload)
        key, spec = self._tray_profiles[self._tray_profile_idx % len(self._tray_profiles)]
        self._tray_profile_idx = (self._tray_profile_idx + 1) % len(self._tray_profiles)
        return self._make_payload(key, spec)

    def _set_vision_target(self, pose: dict) -> None:
        self.vision.default_target = TargetPose(x=pose["x_m"], y=pose["y_m"], z=pose.get("z_m", 0.0))

    def _target_contact_z(self) -> float:
        if self._current_target is not None:
            return float(self._current_target.z)
        return float(self._contact_z)

    def _prepare_cycle_targets(self, reset_payload: bool = True) -> None:
        if reset_payload:
            self._active_payload = self._next_payload()
        self._active_payload["is_clean"] = False
        self._flow_stage = "DIRTY_TO_WASHER"
        self._washer_busy = False
        self._washer_done = False
        self._washer_started_ts = 0.0
        self._place_position = dict(self._washer_load)
        self._set_vision_target(self._dirty_pick)

    def _washer_state_dict(self) -> dict:
        remaining = 0.0
        if self._washer_busy:
            remaining = max(0.0, self._washer_cycle_s - (time.time() - self._washer_started_ts))
        return {
            "model": "MIMASA DA-80",
            "mode": "simulated",
            "busy": self._washer_busy,
            "done": self._washer_done,
            "cycle_time_s": self._washer_cycle_s,
            "remaining_s": round(remaining, 1),
            "flow_stage": self._flow_stage,
        }

    async def start(self):
        """doc"""
        if self._running:
            return
        if self.state.state == StateCode.FAULT_LATCHED:
            self.state.add_log(LogLevel.WARN, "CANNOT_START",
                               "Cannot start from FAULT_LATCHED, please reset first")
            return

        self._running = True
        self._paused = False
        self.vision.start_scan()
        self.vision_adapter.start()
        self._cycle_id = self._cycle_id or uuid.uuid4().hex[:10]
        self._task_phase_override = None
        self.state.add_log(LogLevel.INFO, "FSM_STARTED", "FSM started")
        self._task = asyncio.create_task(self._run_loop())

    async def start_cycle(self, task_name: str = "clean_to_conveyor_to_dirty"):
        self._cycle_name = task_name or self._cycle_name
        self._cycle_id = uuid.uuid4().hex[:10]
        self._prepare_cycle_targets(reset_payload=True)
        self.state.task.update({
            "name": self._cycle_name,
            "phase": "WAITING",
            "cycle_id": self._cycle_id,
            "paused": False,
        })
        await self.start()

    async def pause_task(self):
        if not self._running:
            return
        self._paused = True
        self.robot.stop()
        self.robot_adapter.stop_motion()
        self.state.task["paused"] = True
        self.state.add_log(LogLevel.WARN, "TASK_PAUSED", "Task paused by operator")

    async def resume_task(self):
        if not self._running:
            return
        self._paused = False
        self.state.task["paused"] = False
        self.state.add_log(LogLevel.INFO, "TASK_RESUMED", "Task resumed by operator")

    async def abort_task(self):
        self.state.add_log(LogLevel.WARN, "TASK_ABORT", "Task abort requested")
        self._task_phase_override = "WAITING"
        await self.stop()

    async def recovery_search_once(self):
        self._task_phase_override = "RECOVERY_SEARCH"
        self.state.recover_count += 1
        self.state.add_log(LogLevel.WARN, "MANUAL_RECOVERY_SEARCH", "Manual recovery search triggered")

    async def stop(self):
        """doc"""
        self._running = False
        self._paused = False
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
            self._task = None

        self.state.state = StateCode.IDLE
        self.vision_adapter.stop()
        self.vision.reset()
        self.vacuum.turn_off()
        self.robot.stop()
        self.conveyor.stop()
        self._washer_busy = False
        self._washer_done = False
        self._flow_stage = "DIRTY_TO_WASHER"
        self._current_target = None
        self._current_offset_idx = 0
        self._goal_pose = None
        self.state.vacuum_ok = False
        self.state.vacuum_kpa = None
        self.state.grip = {"vacuum_on": False, "sealed": False}
        self.state.vision = {
            "detected": False,
            "confidence": 0.0,
            "tag_id": None,
            "source": "wrist_usb",
            "camera_ok": False,
            "pose_valid": False,
        }
        self.state.task["paused"] = False
        self.state.task["phase"] = "STOPPED"
        self._task_phase_override = "STOPPED"
        self._update_state_for_ws()
        self.state.add_log(LogLevel.INFO, "FSM_STOPPED", "FSM stopped, returned to IDLE")

    async def reset(self):
        """doc"""
        await self.stop()

        # 閲嶇疆鐘讹拷?
        self.state.retry_count = 0
        self.state.recover_count = 0
        self.state.success_count = 0
        self.state.state = StateCode.IDLE
        self.state.vacuum_ok = False
        self.state.vacuum_kpa = None
        self.state.last_event = ""
        self._fault_injected = False

        # 閲嶇疆妯℃嫙锟?
        self.vacuum.reset_faults()
        self.robot = RobotSim()
        self.robot.safe_z = self._safe_pose["z_m"]
        self.robot.x = self._safe_pose["x_m"]
        self.robot.y = self._safe_pose["y_m"]
        self.robot.z = self._safe_pose["z_m"]
        self.grip = GripSim()
        self.vision.reset()
        self.conveyor.stop()
        self.robot_adapter = SimRobotAdapter(self.robot)
        self.vision_adapter = SimVisionAdapter(self.vision)
        self.vacuum_adapter = SimVacuumAdapter(self.vacuum, self.grip)
        self._goal_pose = None
        self._state_timer = 0.0
        self._paused = False
        self._cycle_id = ""
        self._task_phase_override = "WAITING"
        self._prepare_cycle_targets(reset_payload=False)

        # 閲嶇疆 SystemState 锟?3D 瀛楁
        self.state.robot_pose = {"x_m": 0.0, "y_m": 0.0, "z_m": self._safe_pose["z_m"], "roll_deg": 0.0, "pitch_deg": 0.0, "yaw_deg": 0.0}
        self.state.target_pose = self._dirty_pick.copy()
        self.state.place_pose = self._place_position.copy()
        self.state.conveyor = self.conveyor.get_status()
        self.state.vacuum = self.vacuum_adapter.get_status()
        self.state.vacuum["pressure_kpa"] = self.state.vacuum_kpa
        self.state.robot = self.robot_adapter.get_status()
        self.state.task = {
            "name": self._cycle_name,
            "phase": self._derive_task_phase(),
            "cycle_id": self._cycle_id,
            "paused": self._paused,
        }
        self.state.safety = {"web_motion_allowed": False, "manual_jog_enabled": False}
        self.state.calibration = dict(self._calibration_status)
        self.state.payload = dict(self._active_payload)
        self.state.washer = self._washer_state_dict()
        self.state.vision = {
            "detected": False,
            "confidence": 0.0,
            "tag_id": None,
            "source": "wrist_usb",
            "camera_ok": False,
            "pose_valid": False,
        }
        self.state.grip = {"vacuum_on": False, "sealed": False}
        self.state.fault = {"active": False, "code": "", "msg": ""}
        self.state.joint_angles_rad = self.robot.get_joint_angles_rad()
        self.state.conveyor = self.conveyor.get_status()
        self.state.vacuum = self.vacuum_adapter.get_status()
        self.state.vacuum["pressure_kpa"] = self.state.vacuum_kpa
        self.state.robot = self.robot_adapter.get_status()
        self.state.task = {
            "name": self._cycle_name,
            "phase": "WAITING",
            "cycle_id": "",
            "paused": False,
        }
        self.state.safety = {"web_motion_allowed": False, "manual_jog_enabled": False}
        self.state.calibration = dict(self._calibration_status)
        self.state.payload = dict(self._active_payload)
        self.state.washer = self._washer_state_dict()

        self.state.add_log(LogLevel.INFO, "FSM_RESET", "FSM reset to initial state")

    def inject_pre_suction_fail(self):
        """doc"""
        self.vacuum.inject_pre_suction_fail()
        self._fault_injected = True
        self.state.add_log(LogLevel.WARN, "FAULT_INJECT",
                           "Injected: next pre-suction check will fail")

    def inject_drop_once(self):
        """doc"""
        self.vacuum.inject_drop_once()
        self._fault_injected = True
        self.state.add_log(LogLevel.WARN, "FAULT_INJECT",
                           "Injected: will drop during transport")

    def clear_logs(self):
        """doc"""
        self.state.clear_logs()
        self.state.add_log(LogLevel.INFO, "LOG_CLEARED", "Log buffer cleared")

    async def _run_loop(self):
        """doc"""
        last_time = time.time()
        try:
            while self._running:
                current_time = time.time()
                dt = current_time - last_time
                last_time = current_time

                # 闄愬埗 dt 闃叉澶ц烦锟?
                dt = min(dt, 0.1)

                if not self._paused:
                    await self._step(dt)
                self._update_state_for_ws()

                await asyncio.sleep(0.03)  # 30ms tick
        except asyncio.CancelledError:
            pass

    async def _step(self, dt: float):
        """doc"""
        # 鏇存柊瑙嗚鎵弿
        self.vision.update_scan(dt)

        # 骞虫粦绉诲姩鏈哄櫒锟?
        self.robot.step(dt, self._goal_pose)

        # 鏇存柊鐘舵€佽鏃跺櫒
        self._state_timer += dt

        # 鎵ц鐘舵€佸鐞嗗櫒
        handler = {
            StateCode.IDLE: self._handle_idle,
            StateCode.DETECTING_TARGET: self._handle_detecting_target,
            StateCode.PLANNING_APPROACH: self._handle_planning_approach,
            StateCode.MOVING_TO_PREGRASP: self._handle_moving_to_pregrasp,
            StateCode.DESCENDING_TO_CONTACT: self._handle_descending_to_contact,
            StateCode.PRE_SUCTION_CHECK: self._handle_pre_suction_check,
            StateCode.LIFT_VERIFICATION: self._handle_lift_verification,
            StateCode.TRANSPORT_MONITORING: self._handle_transport_monitoring,
            StateCode.MOVING_TO_PLACE: self._handle_moving_to_place,
            StateCode.RELEASING_LOAD: self._handle_releasing_load,
            StateCode.RETURNING_HOME: self._handle_returning_home,
            StateCode.AUTO_RETRYING_GRASP: self._handle_auto_retrying_grasp,
            StateCode.AUTO_RECOVERY_MODE: self._handle_auto_recovery_mode,
            StateCode.FAULT_LATCHED: self._handle_fault_latched,
        }.get(self.state.state)

        if handler:
            await handler(dt)

        # 鏇存柊 GripSim
        ee_at_contact = self._is_ee_at_contact()
        grip_state = self.grip.update(self.state.state.value, ee_at_contact, self._fault_injected, dt)
        self.state.grip = grip_state

        # 鏇存柊鐪熺┖鐘讹拷?
        if self.vacuum.vacuum_on:
            self.state.vacuum_kpa = self.vacuum.read_pressure()
            self.state.vacuum_ok = self.vacuum.vacuum_ok
        else:
            self.state.vacuum_kpa = None
            self.state.vacuum_ok = False

    def _is_ee_at_contact(self) -> bool:
        """doc"""
        if self._goal_pose is None:
            return False
        contact_z = self._goal_pose.get("z_m", 0.3)
        return abs(self.robot.z - contact_z) < 0.01

    def _update_state_for_ws(self):
        """doc"""
        # 鏇存柊 robot_pose
        self.state.robot_pose = self.robot.get_pose_dict()
        self.state.joint_angles_rad = self.robot_adapter.get_joint_angles_rad()

        # 鏇存柊 vision
        vision_data = self.vision_adapter.detect_tray_tag()
        self.state.vision = {
            "detected": bool(vision_data.get("detected", False)),
            "confidence": round(float(vision_data.get("confidence", 0.0)), 2),
            "tag_id": vision_data.get("tag_id"),
            "source": vision_data.get("source", "wrist_usb"),
            "camera_ok": bool(vision_data.get("camera_ok", False)),
            "pose_valid": bool(vision_data.get("pose_valid", False)),
        }
        if vision_data.get("detected") and vision_data.get("target_pose"):
            self.state.target_pose = vision_data["target_pose"]

        # 鏇存柊 place_pose
        self.state.place_pose = self._place_position.copy()
        self.state.conveyor = self.conveyor.get_status()
        self.state.vacuum = self.vacuum_adapter.get_status()
        self.state.vacuum["pressure_kpa"] = self.state.vacuum_kpa
        self.state.robot = self.robot_adapter.get_status()
        self.state.task = {
            "name": self._cycle_name,
            "phase": self._derive_task_phase(),
            "cycle_id": self._cycle_id,
            "paused": self._paused,
        }
        self.state.safety = {"web_motion_allowed": False, "manual_jog_enabled": False}
        self.state.calibration = dict(self._calibration_status)
        self.state.payload = dict(self._active_payload)
        self.state.washer = self._washer_state_dict()
        # 鏇存柊 fault
        if self.state.state == StateCode.FAULT_LATCHED:
            self.state.fault = {
                "active": True,
                "code": "RETRY_EXHAUSTED",
                "msg": "Retry exhausted, manual reset required"
            }
        elif self.state.state == StateCode.AUTO_RECOVERY_MODE:
            self.state.fault = {
                "active": True,
                "code": "RECOVERY",
                "msg": "Auto recovery in progress"
            }
        else:
            self.state.fault = {"active": False, "code": "", "msg": ""}

    def _derive_task_phase(self) -> str:
        if self._task_phase_override:
            return self._task_phase_override
        if self._paused:
            return "PAUSED"
        state = self.state.state
        if state == StateCode.IDLE:
            return "WAITING"
        if self._flow_stage == "WASHER_PROCESS":
            return "WASHER_PROCESS"
        if state in {
            StateCode.DETECTING_TARGET,
            StateCode.PLANNING_APPROACH,
            StateCode.MOVING_TO_PREGRASP,
            StateCode.DESCENDING_TO_CONTACT,
            StateCode.PRE_SUCTION_CHECK,
            StateCode.LIFT_VERIFICATION,
        }:
            return "RACK_PICK" if self._flow_stage == "DIRTY_TO_WASHER" else "RETURN_PICK"
        if state in {
            StateCode.TRANSPORT_MONITORING,
            StateCode.MOVING_TO_PLACE,
            StateCode.RELEASING_LOAD,
        }:
            return "MOVE_TO_WASHER_LOAD" if self._flow_stage == "DIRTY_TO_WASHER" else "MOVE_TO_CLEAN_RACK"
        if state == StateCode.AUTO_RECOVERY_MODE:
            return "RECOVERY_SEARCH"
        if state == StateCode.AUTO_RETRYING_GRASP:
            return "RETRY_GRASP"
        if state == StateCode.FAULT_LATCHED:
            return "FAULT"
        return "WAITING"

    def get_health(self) -> dict:
        return {
            "status": "ok",
            "python_exec": {"up": True},
            "robot": self.robot_adapter.get_status(),
            "camera": self.vision_adapter.get_status(),
            "conveyor": self.conveyor.get_status(),
            "vacuum": self.vacuum_adapter.get_status(),
            "fsm": {
                "running": self._running,
                "paused": self._paused,
                "state": self.state.state.value,
            },
        }

    def get_calibration_status(self) -> dict:
        return dict(self._calibration_status)

    def get_latest_camera_frame_jpeg(self) -> bytes:
        return self.vision_adapter.get_latest_frame_jpeg()

    # ==================== 鐘舵€佸鐞嗗櫒 ====================

    async def _handle_idle(self, dt: float):
        """doc"""
        self._goal_pose = self._safe_pose.copy()
        if self._running and self.state.state == StateCode.IDLE:
            self.state.state = StateCode.DETECTING_TARGET
            self._state_timer = 0.0

    async def _handle_detecting_target(self, dt: float):
        """doc"""
        self._goal_pose = self._safe_pose.copy()

        vision_data = self.vision_adapter.detect_tray_tag()
        if vision_data["detected"] and vision_data["confidence"] > 0.9:
            target = vision_data["target_pose"]
            self._current_target = TargetPose(x=target["x_m"], y=target["y_m"], z=target["z_m"])
            self.state.add_log(LogLevel.INFO, EventCode.TARGET_DETECTED.value,
                               f"Target detected at ({target['x_m']:.2f}, {target['y_m']:.2f})")
            self._current_offset_idx = 0
            self.state.state = StateCode.PLANNING_APPROACH
            self._state_timer = 0.0

    async def _handle_planning_approach(self, dt: float):
        """doc"""
        # 鐭殏寤惰繜鍚庤浆锟?
        if self._state_timer > 0.2:
            self.state.add_log(LogLevel.INFO, EventCode.APPROACH_PLANNED.value,
                               "Approach path planned")
            self.state.state = StateCode.MOVING_TO_PREGRASP
            self._state_timer = 0.0

    async def _handle_moving_to_pregrasp(self, dt: float):
        """doc"""
        if self._current_target:
            dx, dy = GRID_OFFSETS[self._current_offset_idx]
            self._goal_pose = {
                "x_m": self._current_target.x + dx,
                "y_m": self._current_target.y + dy,
                "z_m": self.robot.safe_z
            }

            if self.robot.is_at_goal(self._goal_pose):
                self.state.add_log(LogLevel.INFO, "MOVE_COMPLETE",
                                   f"Arrived at pregrasp position")
                self.state.state = StateCode.DESCENDING_TO_CONTACT
                self._state_timer = 0.0

    async def _handle_descending_to_contact(self, dt: float):
        """doc"""
        if self._current_target:
            dx, dy = GRID_OFFSETS[self._current_offset_idx]
            contact_z = self._target_contact_z()
            self._goal_pose = {
                "x_m": self._current_target.x + dx,
                "y_m": self._current_target.y + dy,
                "z_m": contact_z
            }

            if self.robot.is_at_goal(self._goal_pose):
                self.state.add_log(LogLevel.INFO, "DESCENT_COMPLETE",
                                   f"Descended to contact at z={contact_z:.3f}")
                self.state.state = StateCode.PRE_SUCTION_CHECK
                self._state_timer = 0.0

    async def _handle_pre_suction_check(self, dt: float):
        """doc"""
        if self._state_timer < 0.1:
            return  # 绛夊緟绋冲畾

        if not self.vacuum.vacuum_on:
            self.vacuum.turn_on()
            self.state.add_log(LogLevel.INFO, EventCode.VACUUM_ON.value, "Vacuum turned ON")
            return

        if self._state_timer > 0.3:  # 绛夊緟璐熷帇寤虹珛
            if self.vacuum.vacuum_ok:
                self.state.add_log(LogLevel.INFO, EventCode.PRE_SUCTION_OK.value,
                                   f"Pre-suction OK, pressure={self.state.vacuum_kpa:.1f}kPa")
                self.state.retry_count = 0
                self._fault_injected = False
                self.state.state = StateCode.LIFT_VERIFICATION
            else:
                self.state.add_log(LogLevel.ERROR, EventCode.PRE_SUCTION_FAIL.value,
                                   f"Pre-suction FAILED, pressure={self.state.vacuum_kpa:.1f}kPa")
                self.state.state = StateCode.AUTO_RETRYING_GRASP
            self._state_timer = 0.0

    async def _handle_lift_verification(self, dt: float):
        """doc"""
        if self._current_target:
            dx, dy = GRID_OFFSETS[self._current_offset_idx]
            contact_z = self._target_contact_z()
            self._goal_pose = {
                "x_m": self._current_target.x + dx,
                "y_m": self._current_target.y + dy,
                "z_m": contact_z + 0.12
            }

            if self.robot.is_at_goal(self._goal_pose):
                if self.grip.sealed:
                    self.state.add_log(LogLevel.INFO, EventCode.LIFT_CHECK_OK.value,
                                       "Lift check OK, object held firmly")
                    self.state.state = StateCode.TRANSPORT_MONITORING
                else:
                    self.state.add_log(LogLevel.ERROR, EventCode.LIFT_CHECK_FAIL.value,
                                       "Lift check FAILED, object unstable")
                    self.vacuum.turn_off()
                    self.state.state = StateCode.AUTO_RETRYING_GRASP
                self._state_timer = 0.0

    async def _handle_transport_monitoring(self, dt: float):
        """doc"""
        # 鐩爣锛氭斁缃偣涓婃柟
        source_z = self._target_contact_z()
        transport_z = max(source_z, float(self._place_position.get("z_m", 0.0))) + 0.20
        self._goal_pose = {
            "x_m": self._place_position["x_m"],
            "y_m": self._place_position["y_m"],
            "z_m": transport_z
        }

        # 妫€鏌ユ槸鍚︽帀锟?
        if not self.vacuum.vacuum_ok and self.grip.sealed:
            self.state.add_log(LogLevel.ERROR, EventCode.DROP_DETECTED.value,
                               f"Drop detected during transport!")
            self.state.recover_count += 1
            self.state.state = StateCode.AUTO_RECOVERY_MODE
            self._state_timer = 0.0
            return

        if self.robot.is_at_goal(self._goal_pose):
            self.state.add_log(LogLevel.INFO, "TRANSPORT_COMPLETE",
                               "Transport completed, object stable")
            self.state.state = StateCode.MOVING_TO_PLACE
            self._state_timer = 0.0

    async def _handle_moving_to_place(self, dt: float):
        """doc"""
        self._goal_pose = {
            "x_m": self._place_position["x_m"],
            "y_m": self._place_position["y_m"],
            "z_m": self._place_position["z_m"]
        }

        if self.robot.is_at_goal(self._goal_pose):
            self.state.add_log(LogLevel.INFO, "AT_PLACE_POSITION",
                               f"Arrived at place position")
            self.state.state = StateCode.RELEASING_LOAD
            self._state_timer = 0.0

    async def _handle_releasing_load(self, dt: float):
        """doc"""
        if self._state_timer < 0.1:
            return

        if self.grip.vacuum_on:
            self.vacuum.turn_off()
            self.state.add_log(LogLevel.INFO, EventCode.VACUUM_OFF_RELEASE.value,
                               "Vacuum turned OFF, releasing load")
            return

        if self._state_timer > 0.3:
            if self._flow_stage == "DIRTY_TO_WASHER":
                self._flow_stage = "WASHER_PROCESS"
                self._washer_busy = True
                self._washer_done = False
                self._washer_started_ts = time.time()
                self.conveyor.start()
                self.state.add_log(LogLevel.INFO, "WASHER_START", f"Washer cycle started ({self._washer_cycle_s:.0f}s)")
            elif self._flow_stage == "WASHER_TO_CLEAN":
                self._active_payload["is_clean"] = True
            self.state.state = StateCode.RETURNING_HOME
            self._state_timer = 0.0

    async def _handle_returning_home(self, dt: float):
        """doc"""
        self._goal_pose = self._safe_pose.copy()

        if self.robot.is_at_goal(self._goal_pose):
            if self._flow_stage == "WASHER_PROCESS":
                elapsed = time.time() - self._washer_started_ts if self._washer_started_ts else 0.0
                if elapsed < self._washer_cycle_s:
                    return
                self._washer_busy = False
                self._washer_done = True
                self.conveyor.stop()
                self._flow_stage = "WASHER_TO_CLEAN"
                self._active_payload["is_clean"] = True
                self._place_position = dict(self._clean_place)
                self._set_vision_target(self._washer_return_pick)
                self.vision.start_scan()
                self.state.add_log(LogLevel.INFO, "WASHER_DONE", "Washer cycle complete, target available at return chute")
                self.state.state = StateCode.DETECTING_TARGET
                self._state_timer = 0.0
                return

            self.state.success_count += 1
            self.state.add_log(LogLevel.INFO, EventCode.CYCLE_SUCCESS.value,
                               f"Cycle completed successfully! Total success: {self.state.success_count}")

            # 鍥炲埌 IDLE锛岀瓑寰呬笅娆℃墜鍔ㄥ惎锟?
            self._washer_busy = False
            self._washer_done = False
            self._flow_stage = "DIRTY_TO_WASHER"
            self._running = False
            self.state.state = StateCode.IDLE
            self.state.add_log(LogLevel.INFO, "CYCLE_END", "Returned to IDLE, ready for next start")

    async def _handle_auto_retrying_grasp(self, dt: float):
        """doc"""
        # 鍏堝洖鍒板畨鍏ㄩ珮锟?
        self._goal_pose = {
            "x_m": self.robot.x,
            "y_m": self.robot.y,
            "z_m": self.robot.safe_z
        }

        if self._state_timer < 0.3:
            return  # 绛夊緟绋冲畾

        self.vacuum.turn_off()

        self.state.retry_count += 1

        if self.state.retry_count > MAX_RETRY:
            self.state.add_log(LogLevel.ERROR, EventCode.RETRY_EXHAUSTED.value,
                               f"Retry exhausted after {MAX_RETRY} attempts")
            self.state.state = StateCode.FAULT_LATCHED
            return

        # 璁＄畻涓嬩竴涓亸绉荤储锟?
        self._current_offset_idx = (self.state.retry_count - 1) % len(GRID_OFFSETS)
        dx, dy = GRID_OFFSETS[self._current_offset_idx]

        self.state.add_log(LogLevel.WARN, "RETRY_ATTEMPT",
                           f"Retry #{self.state.retry_count}/9 with offset ({dx*1000:.0f}mm, {dy*1000:.0f}mm)")

        self.state.state = StateCode.MOVING_TO_PREGRASP
        self._state_timer = 0.0

    async def _handle_auto_recovery_mode(self, dt: float):
        """doc"""
        if self._state_timer < 0.1:
            self.state.add_log(LogLevel.WARN, EventCode.RECOVERY_TRIGGERED.value,
                               "Entering auto recovery mode")
            self.robot.stop()

        # 鎶埌瀹夊叏楂樺害
        self._goal_pose = {
            "x_m": self.robot.x,
            "y_m": self.robot.y,
            "z_m": self.robot.safe_z
        }

        if self._state_timer > 0.5:
            self.vacuum.turn_off()

        if self._state_timer > 1.0:
            self.state.add_log(LogLevel.INFO, "RECOVERY_COMPLETE",
                               "Recovery complete, restarting cycle")

            # 閲嶆柊寮€濮嬫锟?
            self._current_offset_idx = 0
            self._fault_injected = False
            self.vision.start_scan()
            self.state.state = StateCode.DETECTING_TARGET
            self._state_timer = 0.0

    async def _handle_fault_latched(self, dt: float):
        """doc"""
        self._running = False
        self._goal_pose = None
        self.vacuum.turn_off()
        self.robot.stop()

        if self.state.last_event != "FAULT_LATCHED_LOGGED":
            self.state.add_log(LogLevel.ERROR, "FAULT_LATCHED_LOGGED",
                               "System in FAULT_LATCHED state, manual reset required")

