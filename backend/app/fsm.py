"""FSM 状态机：真空抓取可靠性与自恢复模块"""
import asyncio
import time
from typing import Optional
from .models import (
    StateCode, EventCode, LogLevel, SystemState,
    GRID_OFFSETS, MAX_RETRY
)
from .simulator import VisionSim, VacuumSim, RobotSim, TargetPose


class GraspFSM:
    """抓取状态机"""

    def __init__(self):
        self.state = SystemState()
        self.vision = VisionSim()
        self.vacuum = VacuumSim()
        self.robot = RobotSim()

        # 运行控制
        self._running: bool = False
        self._task: Optional[asyncio.Task] = None

        # 当前目标位置
        self._current_target: Optional[TargetPose] = None
        self._current_offset_idx: int = 0

        # 放置位置（世界坐标，单位：米）
        self._place_position = (0.0, 0.0, 0.05)

    async def start(self):
        """启动 FSM"""
        if self._running:
            return
        if self.state.state == StateCode.FAULT_LATCHED:
            self.state.add_log(LogLevel.WARN, EventCode.RECOVERY_TRIGGERED.value,
                               "Cannot start from FAULT_LATCHED, please reset first")
            return

        self._running = True
        self.state.add_log(LogLevel.INFO, EventCode.TARGET_DETECTED.value,
                           "FSM started")
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

        # 重置模拟器
        self.vacuum.reset_faults()
        self.robot = RobotSim()

        self.state.add_log(LogLevel.INFO, "FSM_RESET", "FSM reset to initial state")

    def inject_pre_suction_fail(self):
        """注入故障：下一次 PRE_SUCTION_CHECK 失败"""
        self.vacuum.inject_pre_suction_fail()
        self.state.add_log(LogLevel.WARN, "FAULT_INJECT",
                           "Injected: next pre-suction check will fail")

    def inject_drop_once(self):
        """注入故障：TRANSPORT_MONITORING 中掉压"""
        self.vacuum.inject_drop_once()
        self.state.add_log(LogLevel.WARN, "FAULT_INJECT",
                           "Injected: will drop during transport")

    def clear_logs(self):
        """清空日志"""
        self.state.clear_logs()
        self.state.add_log(LogLevel.INFO, "LOG_CLEARED", "Log buffer cleared")

    async def _run_loop(self):
        """主状态机循环"""
        try:
            while self._running:
                await self._step()
                await asyncio.sleep(0.05)  # 50ms 循环周期
        except asyncio.CancelledError:
            pass

    async def _step(self):
        """状态机单步执行"""
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
            await handler()

        # 更新真空状态
        if self.vacuum.vacuum_on:
            self.state.vacuum_kpa = self.vacuum.read_pressure()
            self.state.vacuum_ok = self.vacuum.vacuum_ok
        else:
            self.state.vacuum_kpa = None
            self.state.vacuum_ok = False

    # ==================== 状态处理器 ====================

    async def _handle_idle(self):
        """IDLE: 等待启动"""
        # 从 IDLE 转换到 DETECTING_TARGET 由 start() 触发
        if self._running and self.state.state == StateCode.IDLE:
            self.state.state = StateCode.DETECTING_TARGET

    async def _handle_detecting_target(self):
        """DETECTING_TARGET: 视觉检测目标"""
        await asyncio.sleep(0.2)  # 模拟检测时间

        self._current_target = self.vision.detect_target()
        if self._current_target:
            self.state.add_log(LogLevel.INFO, EventCode.TARGET_DETECTED.value,
                               f"Target detected at ({self._current_target.x:.2f}, {self._current_target.y:.2f})")
            self._current_offset_idx = 0
            self.state.state = StateCode.PLANNING_APPROACH

    async def _handle_planning_approach(self):
        """PLANNING_APPROACH: 规划接近路径"""
        await asyncio.sleep(0.1)  # 模拟规划时间

        self.state.add_log(LogLevel.INFO, EventCode.APPROACH_PLANNED.value,
                           "Approach path planned")
        self.state.state = StateCode.MOVING_TO_PREGRASP

    async def _handle_moving_to_pregrasp(self):
        """MOVING_TO_PREGRASP: 移动到预抓取位置"""
        if self._current_target:
            # 获取当前偏移
            dx, dy = GRID_OFFSETS[self._current_offset_idx]

            # 移动到目标上方（安全高度）
            target_x = self._current_target.x + dx
            target_y = self._current_target.y + dy
            safe_z = self.robot.safe_z

            await asyncio.to_thread(self.robot.move_to, target_x, target_y, safe_z)

            self.state.add_log(LogLevel.INFO, "MOVE_COMPLETE",
                               f"Moved to pregrasp ({target_x:.3f}, {target_y:.3f}, {safe_z:.3f})")
            self.state.state = StateCode.DESCENDING_TO_CONTACT

    async def _handle_descending_to_contact(self):
        """DESCENDING_TO_CONTACT: 下降到接触"""
        # 下降到抓取高度（接近目标表面）
        grasp_z = 0.02  # 20mm
        await asyncio.to_thread(self.robot.move_relative, 0, 0, grasp_z - self.robot.z)

        self.state.add_log(LogLevel.INFO, "DESCENT_COMPLETE",
                           f"Descended to contact at z={grasp_z:.3f}")
        self.state.state = StateCode.PRE_SUCTION_CHECK

    async def _handle_pre_suction_check(self):
        """PRE_SUCTION_CHECK: 预吸取检查"""
        # 开启真空
        self.vacuum.turn_on()
        self.state.add_log(LogLevel.INFO, EventCode.VACUUM_ON.value, "Vacuum turned ON")

        # 等待建立负压（200ms）
        await asyncio.sleep(0.2)

        # 读取压力
        kpa = self.vacuum.read_pressure()
        self.state.vacuum_kpa = kpa
        self.state.vacuum_ok = self.vacuum.vacuum_ok

        if self.state.vacuum_ok:
            self.state.add_log(LogLevel.INFO, EventCode.PRE_SUCTION_OK.value,
                               f"Pre-suction OK, pressure={kpa:.1f}kPa")
            self.state.retry_count = 0  # 重置重试计数
            self.state.state = StateCode.LIFT_VERIFICATION
        else:
            self.state.add_log(LogLevel.ERROR, EventCode.PRE_SUCTION_FAIL.value,
                               f"Pre-suction FAILED, pressure={kpa:.1f}kPa")
            self.state.state = StateCode.AUTO_RETRYING_GRASP

    async def _handle_lift_verification(self):
        """LIFT_VERIFICATION: 抬起验证"""
        # 轻抬 30mm
        lift_height = 0.03
        await asyncio.to_thread(self.robot.move_relative, 0, 0, lift_height)

        # 等待稳定
        await asyncio.sleep(0.1)

        # 再次检查真空
        if self.state.vacuum_ok:
            self.state.add_log(LogLevel.INFO, EventCode.LIFT_CHECK_OK.value,
                               f"Lift check OK, object held firmly")
            self.state.state = StateCode.TRANSPORT_MONITORING
        else:
            self.state.add_log(LogLevel.ERROR, EventCode.LIFT_CHECK_FAIL.value,
                               "Lift check FAILED, object unstable")
            # 放回并重试
            self.vacuum.turn_off()
            await asyncio.sleep(0.1)
            self.state.state = StateCode.AUTO_RETRYING_GRASP

    async def _handle_transport_monitoring(self):
        """TRANSPORT_MONITORING: 搬运监控"""
        # 计算到放置点的距离
        px, py, pz = self._place_position
        dx = px - self.robot.x
        dy = py - self.robot.y

        # 分步移动并持续监控
        steps = 5
        for i in range(steps):
            if not self._running:
                return

            # 移动一步
            step_x = self.robot.x + dx / steps
            step_y = self.robot.y + dy / steps
            await asyncio.to_thread(self.robot.move_to, step_x, step_y, self.robot.z)

            # 检查真空
            kpa = self.vacuum.read_pressure()
            self.state.vacuum_kpa = kpa
            self.state.vacuum_ok = self.vacuum.vacuum_ok

            if not self.state.vacuum_ok:
                self.state.add_log(LogLevel.ERROR, EventCode.DROP_DETECTED.value,
                                   f"Drop detected during transport! pressure={kpa:.1f}kPa")
                self.state.recover_count += 1
                self.state.state = StateCode.AUTO_RECOVERY_MODE
                return

            await asyncio.sleep(0.1)

        # 到达放置点上方
        self.state.add_log(LogLevel.INFO, "TRANSPORT_COMPLETE",
                           "Transport completed, object stable")
        self.state.state = StateCode.MOVING_TO_PLACE

    async def _handle_moving_to_place(self):
        """MOVING_TO_PLACE: 移动到放置位置"""
        px, py, pz = self._place_position

        # 下降到放置高度
        await asyncio.to_thread(self.robot.move_to, px, py, pz)

        self.state.add_log(LogLevel.INFO, "AT_PLACE_POSITION",
                           f"Arrived at place position ({px:.3f}, {py:.3f}, {pz:.3f})")
        self.state.state = StateCode.RELEASING_LOAD

    async def _handle_releasing_load(self):
        """RELEASING_LOAD: 释放负载"""
        # 关闭真空
        self.vacuum.turn_off()
        self.state.add_log(LogLevel.INFO, EventCode.VACUUM_OFF_RELEASE.value,
                           "Vacuum turned OFF, releasing load")

        await asyncio.sleep(0.2)

        self.state.state = StateCode.RETURNING_HOME

    async def _handle_returning_home(self):
        """RETURNING_HOME: 返回原点"""
        # 抬起到安全高度
        self.robot.go_to_safe_z()

        # 返回原点
        await asyncio.to_thread(self.robot.move_to, 0, 0, self.robot.safe_z)

        self.state.success_count += 1
        self.state.add_log(LogLevel.INFO, EventCode.CYCLE_SUCCESS.value,
                           f"Cycle completed successfully! Total success: {self.state.success_count}")

        # 回到 IDLE，等待下次手动启动
        self._running = False
        self.state.state = StateCode.IDLE
        self.state.add_log(LogLevel.INFO, "CYCLE_END", "Returned to IDLE, ready for next start")

    async def _handle_auto_retrying_grasp(self):
        """AUTO_RETRYING_GRASP: 自动重试抓取"""
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

        # 确保真空关闭
        self.vacuum.turn_off()

        # 回到安全高度后重新接近
        self.robot.go_to_safe_z()
        self.state.state = StateCode.MOVING_TO_PREGRASP

    async def _handle_auto_recovery_mode(self):
        """AUTO_RECOVERY_MODE: 自动恢复模式"""
        self.state.add_log(LogLevel.WARN, EventCode.RECOVERY_TRIGGERED.value,
                           "Entering auto recovery mode")

        # 停止机器人
        self.robot.stop()

        # 确保真空关闭
        self.vacuum.turn_off()

        # 移动到安全高度
        self.robot.go_to_safe_z()

        await asyncio.sleep(0.2)

        self.state.add_log(LogLevel.INFO, "RECOVERY_COMPLETE",
                           "Recovery complete, restarting cycle")

        # 重新开始检测
        self._current_offset_idx = 0
        self.state.state = StateCode.DETECTING_TARGET

    async def _handle_fault_latched(self):
        """FAULT_LATCHED: 故障锁定"""
        # 停止所有操作，等待手动重置
        self._running = False
        self.vacuum.turn_off()
        self.robot.stop()

        # 只在首次进入时记录日志
        if self.state.last_event != "FAULT_LATCHED_LOGGED":
            self.state.add_log(LogLevel.ERROR, "FAULT_LATCHED_LOGGED",
                               "System in FAULT_LATCHED state, manual reset required")
