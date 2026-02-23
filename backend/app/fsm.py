"""FSM 状态机：真空抓取可靠性与自恢复模块（3D可视化版本）"""
import asyncio
import time
from typing import Optional
from .models import (
    StateCode, EventCode, LogLevel, SystemState,
    GRID_OFFSETS, MAX_RETRY
)
from .simulator import VisionSim, VacuumSim, RobotSim, GripSim, TargetPose


class GraspFSM:
    """抓取状态机（3D可视化版本）"""

    def __init__(self):
        self.state = SystemState()
        self.vision = VisionSim()
        self.vacuum = VacuumSim()
        self.robot = RobotSim()
        self.grip = GripSim()

        # 运行控制
        self._running: bool = False
        self._task: Optional[asyncio.Task] = None

        # 当前目标位置
        self._current_target: Optional[TargetPose] = None
        self._current_offset_idx: int = 0

        # 目标位姿（用于平滑移动）
        self._goal_pose: Optional[dict] = None

        # 放置位置（世界坐标，单位：米）
        self._place_position = {"x_m": -0.25, "y_m": 0.25, "z_m": 0.05}

        # 安全位姿
        self._safe_pose = {"x_m": 0.0, "y_m": 0.0, "z_m": 0.3}

        # 接触高度
        self._contact_z = 0.02  # 20mm

        # 状态计时器
        self._state_timer: float = 0.0

        # 故障标志
        self._fault_injected: bool = False

    async def start(self):
        """启动 FSM"""
        if self._running:
            return
        if self.state.state == StateCode.FAULT_LATCHED:
            self.state.add_log(LogLevel.WARN, "CANNOT_START",
                               "Cannot start from FAULT_LATCHED, please reset first")
            return

        self._running = True
        self.vision.start_scan()
        self.state.add_log(LogLevel.INFO, "FSM_STARTED", "FSM started")
        self._task = asyncio.create_task(self._run_loop())

    async def stop(self):
        """停止 FSM"""
        self._running = False
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
            self._task = None

        self.state.state = StateCode.IDLE
        self.vacuum.turn_off()
        self.robot.stop()
        self._goal_pose = None
        self.state.add_log(LogLevel.INFO, "FSM_STOPPED", "FSM stopped, returned to IDLE")

    async def reset(self):
        """重置 FSM"""
        await self.stop()

        # 重置状态
        self.state.retry_count = 0
        self.state.recover_count = 0
        self.state.success_count = 0
        self.state.state = StateCode.IDLE
        self.state.vacuum_ok = False
        self.state.vacuum_kpa = None
        self.state.last_event = ""
        self._fault_injected = False

        # 重置模拟器
        self.vacuum.reset_faults()
        self.robot = RobotSim()
        self.grip = GripSim()
        self.vision.reset()
        self._goal_pose = None
        self._state_timer = 0.0

        # 重置 SystemState 的 3D 字段
        self.state.robot_pose = {"x_m": 0.0, "y_m": 0.0, "z_m": 0.3, "roll_deg": 0.0, "pitch_deg": 0.0, "yaw_deg": 0.0}
        self.state.target_pose = {"x_m": 0.35, "y_m": 0.12, "z_m": 0.0}
        self.state.place_pose = self._place_position.copy()
        self.state.vision = {"detected": False, "confidence": 0.0}
        self.state.grip = {"vacuum_on": False, "sealed": False}
        self.state.fault = {"active": False, "code": "", "msg": ""}

        self.state.add_log(LogLevel.INFO, "FSM_RESET", "FSM reset to initial state")

    def inject_pre_suction_fail(self):
        """注入故障：下一次 PRE_SUCTION_CHECK 失败"""
        self.vacuum.inject_pre_suction_fail()
        self._fault_injected = True
        self.state.add_log(LogLevel.WARN, "FAULT_INJECT",
                           "Injected: next pre-suction check will fail")

    def inject_drop_once(self):
        """注入故障：TRANSPORT_MONITORING 中掉压"""
        self.vacuum.inject_drop_once()
        self._fault_injected = True
        self.state.add_log(LogLevel.WARN, "FAULT_INJECT",
                           "Injected: will drop during transport")

    def clear_logs(self):
        """清空日志"""
        self.state.clear_logs()
        self.state.add_log(LogLevel.INFO, "LOG_CLEARED", "Log buffer cleared")

    async def _run_loop(self):
        """主状态机循环（30~50ms tick）"""
        last_time = time.time()
        try:
            while self._running:
                current_time = time.time()
                dt = current_time - last_time
                last_time = current_time

                # 限制 dt 防止大跳跃
                dt = min(dt, 0.1)

                await self._step(dt)
                self._update_state_for_ws()

                await asyncio.sleep(0.03)  # 30ms tick
        except asyncio.CancelledError:
            pass

    async def _step(self, dt: float):
        """状态机单步执行"""
        # 更新视觉扫描
        self.vision.update_scan(dt)

        # 平滑移动机器人
        self.robot.step(dt, self._goal_pose)

        # 更新状态计时器
        self._state_timer += dt

        # 执行状态处理器
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

        # 更新 GripSim
        ee_at_contact = self._is_ee_at_contact()
        grip_state = self.grip.update(self.state.state.value, ee_at_contact, self._fault_injected, dt)
        self.state.grip = grip_state

        # 更新真空状态
        if self.vacuum.vacuum_on:
            self.state.vacuum_kpa = self.vacuum.read_pressure()
            self.state.vacuum_ok = self.vacuum.vacuum_ok
        else:
            self.state.vacuum_kpa = None
            self.state.vacuum_ok = False

    def _is_ee_at_contact(self) -> bool:
        """判断末端是否在接触位置"""
        if self._goal_pose is None:
            return False
        contact_z = self._goal_pose.get("z_m", 0.3)
        return abs(self.robot.z - contact_z) < 0.01 and contact_z < 0.1

    def _update_state_for_ws(self):
        """更新 SystemState 用于 WS 推送"""
        # 更新 robot_pose
        self.state.robot_pose = self.robot.get_pose_dict()

        # 更新 vision
        vision_data = self.vision.detect()
        self.state.vision = {
            "detected": vision_data["detected"],
            "confidence": round(vision_data["confidence"], 2)
        }
        if vision_data["detected"]:
            self.state.target_pose = vision_data["target_pose"]

        # 更新 place_pose
        self.state.place_pose = self._place_position.copy()

        # 更新 fault
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

    # ==================== 状态处理器 ====================

    async def _handle_idle(self, dt: float):
        """IDLE: 等待启动"""
        self._goal_pose = self._safe_pose.copy()
        if self._running and self.state.state == StateCode.IDLE:
            self.state.state = StateCode.DETECTING_TARGET
            self._state_timer = 0.0

    async def _handle_detecting_target(self, dt: float):
        """DETECTING_TARGET: 视觉检测目标"""
        self._goal_pose = self._safe_pose.copy()

        vision_data = self.vision.detect()
        if vision_data["detected"] and vision_data["confidence"] > 0.9:
            target = vision_data["target_pose"]
            self._current_target = TargetPose(x=target["x_m"], y=target["y_m"], z=target["z_m"])
            self.state.add_log(LogLevel.INFO, EventCode.TARGET_DETECTED.value,
                               f"Target detected at ({target['x_m']:.2f}, {target['y_m']:.2f})")
            self._current_offset_idx = 0
            self.state.state = StateCode.PLANNING_APPROACH
            self._state_timer = 0.0

    async def _handle_planning_approach(self, dt: float):
        """PLANNING_APPROACH: 规划接近路径"""
        # 短暂延迟后转换
        if self._state_timer > 0.2:
            self.state.add_log(LogLevel.INFO, EventCode.APPROACH_PLANNED.value,
                               "Approach path planned")
            self.state.state = StateCode.MOVING_TO_PREGRASP
            self._state_timer = 0.0

    async def _handle_moving_to_pregrasp(self, dt: float):
        """MOVING_TO_PREGRASP: 移动到预抓取位置"""
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
        """DESCENDING_TO_CONTACT: 下降到接触"""
        if self._current_target:
            dx, dy = GRID_OFFSETS[self._current_offset_idx]
            self._goal_pose = {
                "x_m": self._current_target.x + dx,
                "y_m": self._current_target.y + dy,
                "z_m": self._contact_z
            }

            if self.robot.is_at_goal(self._goal_pose):
                self.state.add_log(LogLevel.INFO, "DESCENT_COMPLETE",
                                   f"Descended to contact at z={self._contact_z:.3f}")
                self.state.state = StateCode.PRE_SUCTION_CHECK
                self._state_timer = 0.0

    async def _handle_pre_suction_check(self, dt: float):
        """PRE_SUCTION_CHECK: 预吸取检查"""
        if self._state_timer < 0.1:
            return  # 等待稳定

        if not self.vacuum.vacuum_on:
            self.vacuum.turn_on()
            self.state.add_log(LogLevel.INFO, EventCode.VACUUM_ON.value, "Vacuum turned ON")
            return

        if self._state_timer > 0.3:  # 等待负压建立
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
        """LIFT_VERIFICATION: 抬起验证"""
        if self._current_target:
            dx, dy = GRID_OFFSETS[self._current_offset_idx]
            self._goal_pose = {
                "x_m": self._current_target.x + dx,
                "y_m": self._current_target.y + dy,
                "z_m": self._contact_z + 0.05  # 抬起 50mm
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
        """TRANSPORT_MONITORING: 搬运监控"""
        # 目标：放置点上方
        self._goal_pose = {
            "x_m": self._place_position["x_m"],
            "y_m": self._place_position["y_m"],
            "z_m": 0.15  # 运输高度
        }

        # 检查是否掉压
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
        """MOVING_TO_PLACE: 移动到放置位置"""
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
        """RELEASING_LOAD: 释放负载"""
        if self._state_timer < 0.1:
            return

        if self.grip.vacuum_on:
            self.vacuum.turn_off()
            self.state.add_log(LogLevel.INFO, EventCode.VACUUM_OFF_RELEASE.value,
                               "Vacuum turned OFF, releasing load")
            return

        if self._state_timer > 0.3:
            self.state.state = StateCode.RETURNING_HOME
            self._state_timer = 0.0

    async def _handle_returning_home(self, dt: float):
        """RETURNING_HOME: 返回原点"""
        self._goal_pose = self._safe_pose.copy()

        if self.robot.is_at_goal(self._goal_pose):
            self.state.success_count += 1
            self.state.add_log(LogLevel.INFO, EventCode.CYCLE_SUCCESS.value,
                               f"Cycle completed successfully! Total success: {self.state.success_count}")

            # 回到 IDLE，等待下次手动启动
            self._running = False
            self.state.state = StateCode.IDLE
            self.state.add_log(LogLevel.INFO, "CYCLE_END", "Returned to IDLE, ready for next start")

    async def _handle_auto_retrying_grasp(self, dt: float):
        """AUTO_RETRYING_GRASP: 自动重试抓取"""
        # 先回到安全高度
        self._goal_pose = {
            "x_m": self.robot.x,
            "y_m": self.robot.y,
            "z_m": self.robot.safe_z
        }

        if self._state_timer < 0.3:
            return  # 等待稳定

        self.vacuum.turn_off()

        self.state.retry_count += 1

        if self.state.retry_count > MAX_RETRY:
            self.state.add_log(LogLevel.ERROR, EventCode.RETRY_EXHAUSTED.value,
                               f"Retry exhausted after {MAX_RETRY} attempts")
            self.state.state = StateCode.FAULT_LATCHED
            return

        # 计算下一个偏移索引
        self._current_offset_idx = (self.state.retry_count - 1) % len(GRID_OFFSETS)
        dx, dy = GRID_OFFSETS[self._current_offset_idx]

        self.state.add_log(LogLevel.WARN, "RETRY_ATTEMPT",
                           f"Retry #{self.state.retry_count}/9 with offset ({dx*1000:.0f}mm, {dy*1000:.0f}mm)")

        self.state.state = StateCode.MOVING_TO_PREGRASP
        self._state_timer = 0.0

    async def _handle_auto_recovery_mode(self, dt: float):
        """AUTO_RECOVERY_MODE: 自动恢复模式"""
        if self._state_timer < 0.1:
            self.state.add_log(LogLevel.WARN, EventCode.RECOVERY_TRIGGERED.value,
                               "Entering auto recovery mode")
            self.robot.stop()

        # 抬到安全高度
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

            # 重新开始检测
            self._current_offset_idx = 0
            self._fault_injected = False
            self.vision.start_scan()
            self.state.state = StateCode.DETECTING_TARGET
            self._state_timer = 0.0

    async def _handle_fault_latched(self, dt: float):
        """FAULT_LATCHED: 故障锁定"""
        self._running = False
        self._goal_pose = None
        self.vacuum.turn_off()
        self.robot.stop()

        if self.state.last_event != "FAULT_LATCHED_LOGGED":
            self.state.add_log(LogLevel.ERROR, "FAULT_LATCHED_LOGGED",
                               "System in FAULT_LATCHED state, manual reset required")
