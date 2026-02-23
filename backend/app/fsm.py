"""FSM state machine driven by URDF joint angles."""
import asyncio
import time
from typing import Optional

from .models import StateCode, EventCode, LogLevel, SystemState, GRID_OFFSETS, MAX_RETRY
from .simulator import VisionSim, VacuumSim, GripSim, JointRobotSim


class GraspFSM:
    """Grasp FSM with joint angle outputs for URDF frontend."""

    def __init__(self):
        self.state = SystemState()
        self.vision = VisionSim()
        self.vacuum = VacuumSim()
        self.grip = GripSim()
        self.joint_robot = JointRobotSim()

        self._running: bool = False
        self._task: Optional[asyncio.Task] = None

        self._state_timer: float = 0.0
        self._fault_injected: bool = False
        self._recovery_logged: bool = False

        self._target_pose = {"x_m": 0.35, "y_m": 0.12, "z_m": 0.0}
        self._place_pose = {"x_m": -0.25, "y_m": 0.25, "z_m": 0.0}
        self._retry_index: int = 0

        # 6-axis target templates in radians.
        self.poses = {
            "SAFE": [0.0, -1.1, 1.6, 0.6, 1.57, 0.0],
            "SCAN": [0.35, -1.05, 1.45, 0.7, 1.57, 0.25],
            "PREGRASP": [0.25, -1.25, 1.85, 0.45, 1.57, 0.30],
            "CONTACT": [0.25, -1.52, 2.10, 0.25, 1.57, 0.30],
            "LIFT": [0.25, -1.15, 1.70, 0.55, 1.57, 0.30],
            "TRANSPORT": [-0.40, -1.00, 1.30, 0.90, 1.57, -0.55],
            "PLACE_PRE": [-0.80, -1.08, 1.42, 0.75, 1.57, -0.80],
            "PLACE_DOWN": [-0.80, -1.33, 1.78, 0.46, 1.57, -0.80],
            "RECOVERY_UP": [0.0, -0.85, 1.15, 1.05, 1.57, 0.0],
        }

    async def start(self):
        if self._running:
            return
        if self.state.state == StateCode.FAULT_LATCHED:
            self.state.add_log(LogLevel.WARN, "CANNOT_START", "Cannot start from FAULT_LATCHED, please reset first")
            return

        self._running = True
        self.vision.start_scan()
        self._state_timer = 0.0
        self._recovery_logged = False
        self.state.add_log(LogLevel.INFO, "FSM_STARTED", "FSM started")
        self._task = asyncio.create_task(self._run_loop())

    async def stop(self):
        self._running = False
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
            self._task = None

        self.vacuum.turn_off()
        self.grip.reset()
        self.state.state = StateCode.IDLE
        self._set_pose("SAFE")
        self.state.add_log(LogLevel.INFO, "FSM_STOPPED", "FSM stopped, returned to IDLE")

    async def reset(self):
        await self.stop()
        self.state.retry_count = 0
        self.state.recover_count = 0
        self.state.success_count = 0
        self.state.last_event = ""
        self.state.vacuum_kpa = None
        self.state.vacuum_ok = False

        self.vacuum.reset_faults()
        self.grip.reset()
        self.vision.reset()
        self.joint_robot.reset()

        self._fault_injected = False
        self._retry_index = 0
        self._state_timer = 0.0
        self._recovery_logged = False

        self.state.state = StateCode.IDLE
        self.state.target_pose = self._target_pose.copy()
        self.state.place_pose = self._place_pose.copy()
        self.state.vision = {"detected": False, "confidence": 0.0}
        self.state.grip = {"vacuum_on": False, "sealed": False}
        self.state.fault = {"active": False, "code": "", "msg": ""}
        self.state.joint_angles_rad = self.joint_robot.get_joint_angles()
        self.state.robot_pose = self.joint_robot.get_pose_dict()

        self.state.add_log(LogLevel.INFO, "FSM_RESET", "FSM reset to initial state")

    def inject_pre_suction_fail(self):
        self.vacuum.inject_pre_suction_fail()
        self._fault_injected = True
        self.state.add_log(LogLevel.WARN, "FAULT_INJECT", "Injected: next pre-suction check will fail")

    def inject_drop_once(self):
        self.vacuum.inject_drop_once()
        self._fault_injected = True
        self.state.add_log(LogLevel.WARN, "FAULT_INJECT", "Injected: will drop during transport")

    def clear_logs(self):
        self.state.clear_logs()
        self.state.add_log(LogLevel.INFO, "LOG_CLEARED", "Log buffer cleared")

    async def _run_loop(self):
        last_time = time.time()
        try:
            while self._running:
                now = time.time()
                dt = min(now - last_time, 0.1)
                last_time = now

                await self._step(dt)
                await asyncio.sleep(0.03)  # ~33Hz
        except asyncio.CancelledError:
            pass

    def _set_pose(self, pose_name: str):
        target = self.poses[pose_name].copy()
        if pose_name in ("PREGRASP", "CONTACT", "LIFT") and self._retry_index > 0:
            dx, dy = GRID_OFFSETS[self._retry_index % len(GRID_OFFSETS)]
            target[0] += dx * 4.0
            target[5] += dy * 5.0
        self.joint_robot.set_target(target)

    def _update_ws_state(self, dt: float):
        self.joint_robot.step(dt)
        self.state.joint_angles_rad = self.joint_robot.get_joint_angles()
        self.state.robot_pose = self.joint_robot.get_pose_dict()  # keep backward compatibility

        vision_data = self.vision.detect()
        self.state.vision = {
            "detected": vision_data["detected"],
            "confidence": round(vision_data["confidence"], 2),
        }
        if vision_data["detected"]:
            self._target_pose = vision_data["target_pose"]
        self.state.target_pose = self._target_pose.copy()
        self.state.place_pose = self._place_pose.copy()

        if self.vacuum.vacuum_on:
            self.state.vacuum_kpa = self.vacuum.read_pressure()
            self.state.vacuum_ok = self.vacuum.vacuum_ok
        else:
            self.state.vacuum_kpa = None
            self.state.vacuum_ok = False

        ee_at_contact = self.state.state in (StateCode.DESCENDING_TO_CONTACT, StateCode.PRE_SUCTION_CHECK) and self.joint_robot.is_at_target(0.03)
        self.state.grip = self.grip.update(self.state.state.value, ee_at_contact, self._fault_injected, dt)

        if self.state.state == StateCode.FAULT_LATCHED:
            self.state.fault = {"active": True, "code": "RETRY_EXHAUSTED", "msg": "Retry exhausted, manual reset required"}
        elif self.state.state == StateCode.AUTO_RECOVERY_MODE:
            self.state.fault = {"active": True, "code": "AUTO_RECOVERY_MODE", "msg": "Recovery in progress"}
        else:
            self.state.fault = {"active": False, "code": "", "msg": ""}

    async def _step(self, dt: float):
        self._state_timer += dt
        self.vision.update_scan(dt)

        if self.state.state == StateCode.IDLE:
            self._set_pose("SAFE")
            if self._running:
                self.state.state = StateCode.DETECTING_TARGET
                self._state_timer = 0.0

        elif self.state.state == StateCode.DETECTING_TARGET:
            self._set_pose("SCAN")
            if self.state.vision.get("detected") and self.state.vision.get("confidence", 0.0) > 0.90:
                self.state.add_log(LogLevel.INFO, EventCode.TARGET_DETECTED.value,
                                   f"Target detected at ({self._target_pose['x_m']:.2f}, {self._target_pose['y_m']:.2f})")
                self.state.state = StateCode.PLANNING_APPROACH
                self._state_timer = 0.0

        elif self.state.state == StateCode.PLANNING_APPROACH:
            self._set_pose("SCAN")
            if self._state_timer >= 0.2:
                self.state.add_log(LogLevel.INFO, EventCode.APPROACH_PLANNED.value, "Approach planned")
                self.state.state = StateCode.MOVING_TO_PREGRASP
                self._state_timer = 0.0

        elif self.state.state == StateCode.MOVING_TO_PREGRASP:
            self._set_pose("PREGRASP")
            if self.joint_robot.is_at_target():
                self.state.state = StateCode.DESCENDING_TO_CONTACT
                self._state_timer = 0.0

        elif self.state.state == StateCode.DESCENDING_TO_CONTACT:
            self._set_pose("CONTACT")
            if self.joint_robot.is_at_target() and self._state_timer > 0.15:
                self.state.state = StateCode.PRE_SUCTION_CHECK
                self._state_timer = 0.0

        elif self.state.state == StateCode.PRE_SUCTION_CHECK:
            self._set_pose("CONTACT")
            if not self.vacuum.vacuum_on:
                self.vacuum.turn_on()
                self.state.add_log(LogLevel.INFO, EventCode.VACUUM_ON.value, "Vacuum turned ON")

            if self._state_timer > 0.35:
                if self.vacuum.vacuum_ok:
                    self.state.add_log(LogLevel.INFO, EventCode.PRE_SUCTION_OK.value, "Pre-suction OK")
                    self._fault_injected = False
                    self.state.retry_count = 0
                    self.state.state = StateCode.LIFT_VERIFICATION
                else:
                    self.state.add_log(LogLevel.ERROR, EventCode.PRE_SUCTION_FAIL.value, "Pre-suction FAILED")
                    self.state.state = StateCode.AUTO_RETRYING_GRASP
                self._state_timer = 0.0

        elif self.state.state == StateCode.LIFT_VERIFICATION:
            self._set_pose("LIFT")
            if self.joint_robot.is_at_target() and self._state_timer > 0.2:
                if self.state.grip.get("sealed"):
                    self.state.add_log(LogLevel.INFO, EventCode.LIFT_CHECK_OK.value, "Lift check OK")
                    self.state.state = StateCode.TRANSPORT_MONITORING
                else:
                    self.state.add_log(LogLevel.ERROR, EventCode.LIFT_CHECK_FAIL.value, "Lift check FAILED")
                    self.vacuum.turn_off()
                    self.state.state = StateCode.AUTO_RETRYING_GRASP
                self._state_timer = 0.0

        elif self.state.state == StateCode.TRANSPORT_MONITORING:
            self._set_pose("TRANSPORT")
            if self.state.grip.get("sealed") and not self.vacuum.vacuum_ok:
                self.state.add_log(LogLevel.ERROR, EventCode.DROP_DETECTED.value, "Drop detected during transport")
                self.state.recover_count += 1
                self.state.state = StateCode.AUTO_RECOVERY_MODE
                self._state_timer = 0.0
            elif self.joint_robot.is_at_target() and self._state_timer > 0.2:
                self.state.state = StateCode.MOVING_TO_PLACE
                self._state_timer = 0.0

        elif self.state.state == StateCode.MOVING_TO_PLACE:
            self._set_pose("PLACE_PRE")
            if self.joint_robot.is_at_target() and self._state_timer > 0.2:
                self.state.state = StateCode.RELEASING_LOAD
                self._state_timer = 0.0

        elif self.state.state == StateCode.RELEASING_LOAD:
            self._set_pose("PLACE_DOWN")
            if self.joint_robot.is_at_target() and self._state_timer > 0.25:
                self.vacuum.turn_off()
                self.state.add_log(LogLevel.INFO, EventCode.VACUUM_OFF_RELEASE.value, "Vacuum OFF and release")
                self.state.state = StateCode.RETURNING_HOME
                self._state_timer = 0.0

        elif self.state.state == StateCode.RETURNING_HOME:
            self._set_pose("SAFE")
            if self.joint_robot.is_at_target() and self._state_timer > 0.3:
                self.state.success_count += 1
                self.state.add_log(LogLevel.INFO, EventCode.CYCLE_SUCCESS.value,
                                   f"Cycle success, total={self.state.success_count}")
                self._running = False
                self.state.state = StateCode.IDLE
                self._state_timer = 0.0

        elif self.state.state == StateCode.AUTO_RETRYING_GRASP:
            self._set_pose("SAFE")
            if self.joint_robot.is_at_target() and self._state_timer > 0.25:
                self.vacuum.turn_off()
                self.state.retry_count += 1
                if self.state.retry_count > MAX_RETRY:
                    self.state.add_log(LogLevel.ERROR, EventCode.RETRY_EXHAUSTED.value,
                                       f"Retry exhausted after {MAX_RETRY}")
                    self.state.state = StateCode.FAULT_LATCHED
                else:
                    self._retry_index = (self.state.retry_count - 1) % len(GRID_OFFSETS)
                    dx, dy = GRID_OFFSETS[self._retry_index]
                    self.state.add_log(LogLevel.WARN, "RETRY_ATTEMPT",
                                       f"Retry {self.state.retry_count}/{MAX_RETRY} offset=({dx*1000:.0f}mm,{dy*1000:.0f}mm)")
                    self.state.state = StateCode.MOVING_TO_PREGRASP
                self._state_timer = 0.0

        elif self.state.state == StateCode.AUTO_RECOVERY_MODE:
            self._set_pose("RECOVERY_UP")
            if not self._recovery_logged:
                self.state.add_log(LogLevel.WARN, EventCode.RECOVERY_TRIGGERED.value, "Entering AUTO_RECOVERY_MODE")
                self._recovery_logged = True

            if self._state_timer > 0.5:
                self.vacuum.turn_off()

            if self.joint_robot.is_at_target() and self._state_timer > 1.1:
                self._fault_injected = False
                self._retry_index = 0
                self._recovery_logged = False
                self.vision.start_scan()
                self.state.add_log(LogLevel.INFO, "RECOVERY_COMPLETE", "Recovery done, restart detect")
                self.state.state = StateCode.DETECTING_TARGET
                self._state_timer = 0.0

        elif self.state.state == StateCode.FAULT_LATCHED:
            self._set_pose("SAFE")
            self.vacuum.turn_off()
            self._running = False
            if self.state.last_event != "FAULT_LATCHED_LOGGED":
                self.state.add_log(LogLevel.ERROR, "FAULT_LATCHED_LOGGED", "Manual reset required")

        self._update_ws_state(dt)
