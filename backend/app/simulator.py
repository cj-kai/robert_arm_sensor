"""模拟器：VisionSim, VacuumSim, RobotSim, GripSim, JointRobotSim"""
import random
import time
import math
from typing import Optional, Tuple, List, Dict
from dataclasses import dataclass


@dataclass
class TargetPose:
    """目标位姿（世界坐标，单位：米）"""
    x: float
    y: float
    z: float = 0.0


class VisionSim:
    """视觉模拟器：返回检测状态和目标点"""

    def __init__(self):
        # 固定目标点（世界坐标，单位：米）
        self.default_target = TargetPose(x=0.35, y=0.12, z=0.0)
        self._detected = False
        self._confidence = 0.0
        self._scan_time = 0.0

    def detect(self) -> dict:
        """检测目标位置，返回 dict"""
        # 模拟检测过程（逐步增加 confidence）
        if self._scan_time < 0.5:
            self._detected = False
            self._confidence = min(0.95, self._scan_time * 2)
        else:
            self._detected = True
            self._confidence = 0.85 + 0.1 * random.random()

        return {
            "detected": self._detected,
            "confidence": self._confidence,
            "tag_id": 1 if self._detected else None,
            "pose_valid": self._detected,
            "camera_ok": True,
            "source": "wrist_usb",
            "target_pose": {
                "x_m": self.default_target.x,
                "y_m": self.default_target.y,
                "z_m": self.default_target.z
            }
        }

    def start_scan(self):
        """开始扫描"""
        self._scan_time = 0.0
        self._detected = False
        self._confidence = 0.0

    def update_scan(self, dt: float):
        """更新扫描时间"""
        self._scan_time += dt

    def reset(self):
        """重置"""
        self._detected = False
        self._confidence = 0.0
        self._scan_time = 0.0


class VacuumSim:
    """真空模拟器：支持故障注入"""

    # 阈值定义（单位：kPa）
    V_HOLD = -45.0  # 保持阈值：vacuum_ok 需要低于此值
    V_DROP = -30.0  # 掉压阈值

    # 正常压力范围
    V_NORMAL_MIN = -60.0
    V_NORMAL_MAX = -50.0

    def __init__(self):
        self.vacuum_on: bool = False
        self.vacuum_kpa: float = 0.0  # 大气压初始值

        # 故障注入标志
        self._next_pre_suction_fail: bool = False
        self._drop_once_triggered: bool = False
        self._drop_once_pending: bool = False

    def turn_on(self):
        """开启真空"""
        self.vacuum_on = True
        if self._next_pre_suction_fail:
            # 故障注入：设置低压
            self.vacuum_kpa = self.V_DROP + 5  # -25kPa，高于保持阈值
            self._next_pre_suction_fail = False
        else:
            # 正常建立负压
            self.vacuum_kpa = random.uniform(self.V_NORMAL_MIN, self.V_NORMAL_MAX)

    def turn_off(self):
        """关闭真空（释放）"""
        self.vacuum_on = False
        self.vacuum_kpa = 0.0  # 恢复大气压

    @property
    def vacuum_ok(self) -> bool:
        """判断真空是否正常：vacuum_ok = (vacuum_kpa <= V_HOLD)"""
        if not self.vacuum_on:
            return False
        return self.vacuum_kpa <= self.V_HOLD

    def read_pressure(self) -> float:
        """读取当前压力值"""
        if not self.vacuum_on:
            return 0.0

        # 如果有待触发的掉压
        if self._drop_once_pending:
            self._drop_once_triggered = True
            self._drop_once_pending = False
            self.vacuum_kpa = self.V_DROP + 5  # -25kPa
            return self.vacuum_kpa

        # 正常状态：小幅波动
        if self.vacuum_ok:
            noise = random.uniform(-1.0, 1.0)
            self.vacuum_kpa = max(
                self.V_NORMAL_MIN,
                min(self.V_NORMAL_MAX, self.vacuum_kpa + noise)
            )

        return self.vacuum_kpa

    def inject_pre_suction_fail(self):
        """注入故障：下一次 PRE_SUCTION_CHECK 必失败"""
        self._next_pre_suction_fail = True

    def inject_drop_once(self):
        """注入故障：下一次 TRANSPORT_MONITORING 中触发掉压"""
        self._drop_once_pending = True

    def reset_faults(self):
        """重置故障注入"""
        self._next_pre_suction_fail = False
        self._drop_once_pending = False
        self._drop_once_triggered = False


class RobotSim:
    """机器人模拟器：支持平滑移动和实时位姿更新"""

    def __init__(self):
        # 当前位姿（世界坐标，单位：米）
        self.x: float = 0.0
        self.y: float = 0.0
        self.z: float = 1.0  # phase-1 tray washer layout safe standby height

        # 姿态（欧拉角，单位：度）
        self.roll: float = 0.0
        self.pitch: float = 0.0
        self.yaw: float = 0.0

        # 安全高度
        self.safe_z: float = 1.0

        # 动作日志
        self.action_log: List[str] = []

        # 移动速度（米/秒）
        self.default_speed: float = 0.35

    def step(self, dt: float, goal_pose: Optional[dict], speed_mps: float = None):
        """平滑移动到目标位姿（每 tick 调用一次）"""
        if goal_pose is None:
            return

        if speed_mps is None:
            speed_mps = self.default_speed

        dx = goal_pose.get("x_m", self.x) - self.x
        dy = goal_pose.get("y_m", self.y) - self.y
        dz = goal_pose.get("z_m", self.z) - self.z

        dist = (dx*dx + dy*dy + dz*dz) ** 0.5

        if dist > 0.001:  # 1mm threshold
            step_dist = min(speed_mps * dt, dist)
            self.x += dx / dist * step_dist
            self.y += dy / dist * step_dist
            self.z += dz / dist * step_dist

        # 更新姿态
        if "roll_deg" in goal_pose:
            self.roll = goal_pose["roll_deg"]
        if "pitch_deg" in goal_pose:
            self.pitch = goal_pose["pitch_deg"]
        if "yaw_deg" in goal_pose:
            self.yaw = goal_pose["yaw_deg"]

    def move_to(self, x: float, y: float, z: float):
        """移动到绝对位置（阻塞版本，用于某些场景）"""
        self.action_log.append(f"move_to({x:.3f}, {y:.3f}, {z:.3f})")
        self.x, self.y, self.z = x, y, z

    def move_relative(self, dx: float, dy: float, dz: float):
        """相对移动"""
        self.action_log.append(f"move_relative({dx:.3f}, {dy:.3f}, {dz:.3f})")
        self.x += dx
        self.y += dy
        self.z += dz

    def stop(self):
        """停止"""
        self.action_log.append("stop()")

    def go_to_safe_z(self):
        """移动到安全高度"""
        self.action_log.append(f"go_to_safe_z({self.safe_z})")
        self.z = self.safe_z

    def get_pose(self) -> Tuple[float, float, float]:
        """获取当前位姿"""
        return (self.x, self.y, self.z)

    def get_pose_dict(self) -> dict:
        """获取当前位姿（字典格式，用于 WS 推送）"""
        return {
            "x_m": round(self.x, 4),
            "y_m": round(self.y, 4),
            "z_m": round(self.z, 4),
            "roll_deg": round(self.roll, 1),
            "pitch_deg": round(self.pitch, 1),
            "yaw_deg": round(self.yaw, 1)
        }

    def get_joint_angles_rad(self) -> List[float]:
        """Stable simulated 6-axis joint vector (placeholder, not IK-accurate)."""
        x = self.x
        y = self.y
        z = self.z
        j1 = math.atan2(y, x if abs(x) > 1e-6 else 1e-6)
        z_norm = max(-1.0, min(1.0, (z - 0.2) / 0.2))
        r_xy = math.hypot(x, y)
        j2 = -1.1 + 0.5 * z_norm
        j3 = 1.6 - 0.8 * min(1.0, r_xy / 0.5)
        j4 = 0.6 + 0.3 * math.sin(x * 5.0)
        j5 = 1.57 + 0.2 * math.cos(y * 5.0)
        j6 = 0.2 * math.sin((x + y) * 6.0)
        return [round(v, 4) for v in [j1, j2, j3, j4, j5, j6]]

    def is_at_goal(self, goal_pose: dict, threshold: float = 0.005) -> bool:
        """判断是否到达目标位置（阈值 5mm）"""
        if goal_pose is None:
            return True
        dx = goal_pose.get("x_m", self.x) - self.x
        dy = goal_pose.get("y_m", self.y) - self.y
        dz = goal_pose.get("z_m", self.z) - self.z
        dist = (dx*dx + dy*dy + dz*dz) ** 0.5
        return dist < threshold

    def get_last_actions(self, n: int = 10) -> List[str]:
        """获取最近N个动作"""
        return self.action_log[-n:]


class GripSim:
    """夹具模拟器：管理真空吸附状态"""

    def __init__(self):
        self.vacuum_on = False
        self.sealed = False
        self._seal_timer = 0.0
        self._seal_delay = 0.2  # 密封延迟（秒）
        self._release_timer = 0.0
        self._release_delay = 0.3  # 释放延迟（秒）

    def update(self, state: str, ee_at_contact: bool, fault_injected: bool, dt: float) -> dict:
        """根据 FSM 状态和末端位置更新吸附状态

        Args:
            state: 当前 FSM 状态
            ee_at_contact: 末端是否在接触位置
            fault_injected: 是否有故障注入
            dt: 时间步长（秒）

        Returns:
            dict: {"vacuum_on": bool, "sealed": bool}
        """
        from .models import StateCode

        # AUTO_RECOVERY_MODE: 强制 sealed=false，vacuum_on 延迟关闭
        if state == StateCode.AUTO_RECOVERY_MODE.value:
            self.sealed = False
            if self._release_timer < self._release_delay:
                self._release_timer += dt
            else:
                self.vacuum_on = False
            return {"vacuum_on": self.vacuum_on, "sealed": self.sealed}

        # DESCENDING_TO_CONTACT + 到达接触位置: vacuum_on=true
        if state == StateCode.DESCENDING_TO_CONTACT.value or state == StateCode.PRE_SUCTION_CHECK.value:
            if ee_at_contact and not self.vacuum_on:
                self.vacuum_on = True
                self._seal_timer = 0.0

            # 无故障: sealed=true (带延迟)
            if self.vacuum_on and not fault_injected:
                self._seal_timer += dt
                if self._seal_timer >= self._seal_delay:
                    self.sealed = True
            elif fault_injected:
                self.sealed = False

            return {"vacuum_on": self.vacuum_on, "sealed": self.sealed}

        # LIFT_VERIFICATION / TRANSPORT_MONITORING / MOVING_TO_PLACE: 保持吸附
        if state in [StateCode.LIFT_VERIFICATION.value, StateCode.TRANSPORT_MONITORING.value,
                     StateCode.MOVING_TO_PLACE.value]:
            # 如果有掉压故障
            if fault_injected and state == StateCode.TRANSPORT_MONITORING.value:
                self.sealed = False
            return {"vacuum_on": self.vacuum_on, "sealed": self.sealed}

        # RELEASING_LOAD: 释放
        if state == StateCode.RELEASING_LOAD.value:
            self.vacuum_on = False
            self.sealed = False
            self._seal_timer = 0.0
            self._release_timer = 0.0
            return {"vacuum_on": False, "sealed": False}

        # IDLE / 其他状态: 默认关闭
        if state == StateCode.IDLE.value:
            self.vacuum_on = False
            self.sealed = False
            self._seal_timer = 0.0
            self._release_timer = 0.0

        return {"vacuum_on": self.vacuum_on, "sealed": self.sealed}

    def reset(self):
        """重置状态"""
        self.vacuum_on = False
        self.sealed = False
        self._seal_timer = 0.0
        self._release_timer = 0.0


class JointRobotSim:
    """6轴关节机器人模拟器（用于 URDF 关节驱动）"""

    def __init__(self):
        # [q1..q6] in radians
        self.joint_angles = [0.0, -1.1, 1.6, 0.6, 1.57, 0.0]
        self._target_angles = self.joint_angles.copy()
        self.max_speed_rad_s = 1.2

    def set_target(self, target_angles: List[float]):
        """设置目标关节角"""
        if len(target_angles) != 6:
            raise ValueError("target_angles must contain 6 joints")
        self._target_angles = target_angles.copy()

    def step(self, dt: float):
        """按最大角速度插值到目标角"""
        max_step = self.max_speed_rad_s * dt
        for i in range(6):
            err = self._target_angles[i] - self.joint_angles[i]
            if abs(err) <= max_step:
                self.joint_angles[i] = self._target_angles[i]
            else:
                self.joint_angles[i] += max_step if err > 0 else -max_step

    def is_at_target(self, threshold: float = 0.02) -> bool:
        """判断是否到达目标关节角"""
        return all(abs(self._target_angles[i] - self.joint_angles[i]) <= threshold for i in range(6))

    def get_joint_angles(self) -> List[float]:
        """获取当前关节角列表"""
        return [round(q, 4) for q in self.joint_angles]

    def get_pose_dict(self) -> Dict[str, float]:
        """基于前3轴的简化正运动学估算 TCP 位姿（用于兼容现有 UI）"""
        q1, q2, q3, q4, q5, q6 = self.joint_angles
        l1 = 0.28
        l2 = 0.24
        radial = 0.10 + l1 * math.cos(q2) + l2 * math.cos(q2 + q3)
        z = 0.12 + l1 * math.sin(-q2) + l2 * math.sin(-(q2 + q3))
        x = radial * math.cos(q1)
        y = radial * math.sin(q1)
        return {
            "x_m": round(x, 4),
            "y_m": round(y, 4),
            "z_m": round(max(0.02, z), 4),
            "roll_deg": round(math.degrees(q4), 1),
            "pitch_deg": round(math.degrees(q5), 1),
            "yaw_deg": round(math.degrees(q6), 1),
        }

    def reset(self):
        """重置到初始姿态"""
        self.joint_angles = [0.0, -1.1, 1.6, 0.6, 1.57, 0.0]
        self._target_angles = self.joint_angles.copy()
