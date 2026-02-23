"""模拟器：VisionSim, VacuumSim, RobotSim"""
import random
import time
from typing import Optional, Tuple, List
from dataclasses import dataclass


@dataclass
class TargetPose:
    """目标位姿（世界坐标，单位：米）"""
    x: float
    y: float
    z: float = 0.0


class VisionSim:
    """视觉模拟器：返回固定目标点"""

    def __init__(self):
        # 固定目标点（世界坐标，单位：米）
        self.default_target = TargetPose(x=0.30, y=0.10, z=0.0)

    def detect_target(self) -> Optional[TargetPose]:
        """检测目标位置"""
        # 模拟检测延迟
        time.sleep(0.1)
        # 返回固定目标点（后续可接入AruCo）
        return TargetPose(
            x=self.default_target.x,
            y=self.default_target.y,
            z=self.default_target.z
        )


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
    """机器人模拟器：记录动作"""

    def __init__(self):
        # 当前位姿（世界坐标，单位：米）
        self.x: float = 0.0
        self.y: float = 0.0
        self.z: float = 0.3  # 初始高度 300mm

        # 安全高度
        self.safe_z: float = 0.3

        # 动作日志
        self.action_log: List[str] = []

    def move_to(self, x: float, y: float, z: float):
        """移动到绝对位置"""
        self.action_log.append(f"move_to({x:.3f}, {y:.3f}, {z:.3f})")
        self.x, self.y, self.z = x, y, z
        # 模拟移动时间（简化：每100mm约100ms）
        distance = ((x - self.x)**2 + (y - self.y)**2 + (z - self.z)**2) ** 0.5
        time.sleep(min(0.5, distance * 2))

    def move_relative(self, dx: float, dy: float, dz: float):
        """相对移动"""
        self.action_log.append(f"move_relative({dx:.3f}, {dy:.3f}, {dz:.3f})")
        self.x += dx
        self.y += dy
        self.z += dz
        distance = (dx**2 + dy**2 + dz**2) ** 0.5
        time.sleep(min(0.3, distance * 2))

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

    def get_last_actions(self, n: int = 10) -> List[str]:
        """获取最近N个动作"""
        return self.action_log[-n:]
