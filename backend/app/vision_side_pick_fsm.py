"""Vision-guided horizontal side-pick FSM for the tray washer simulation."""
from __future__ import annotations

import math
import random
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Optional

from .ik_solver import CRX20IkPySolver, IKResult, find_default_crx20_urdf_path
from .simulator import RobotSim, VisionSim, VacuumSim


@dataclass
class Vec3:
    x: float
    y: float
    z: float

    def copy(self) -> "Vec3":
        return Vec3(self.x, self.y, self.z)

    def lerp(self, other: "Vec3", t: float) -> "Vec3":
        return Vec3(
            self.x + (other.x - self.x) * t,
            self.y + (other.y - self.y) * t,
            self.z + (other.z - self.z) * t,
        )

    def distance(self, other: "Vec3") -> float:
        dx = self.x - other.x
        dy = self.y - other.y
        dz = self.z - other.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def __add__(self, other: "Vec3") -> "Vec3":
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)


@dataclass
class Quat:
    x: float
    y: float
    z: float
    w: float

    def normalized(self) -> "Quat":
        n = math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w)
        if n <= 1e-9:
            return Quat(0.0, 0.0, 0.0, 1.0)
        return Quat(self.x / n, self.y / n, self.z / n, self.w / n)

    def dot(self, other: "Quat") -> float:
        return self.x * other.x + self.y * other.y + self.z * other.z + self.w * other.w

    def negated(self) -> "Quat":
        return Quat(-self.x, -self.y, -self.z, -self.w)


@dataclass
class Pose:
    pos: Vec3
    quat: Quat

    def copy(self) -> "Pose":
        return Pose(self.pos.copy(), Quat(self.quat.x, self.quat.y, self.quat.z, self.quat.w))


def clamp01(v: float) -> float:
    return max(0.0, min(1.0, v))


def euler_deg_to_quat(roll_deg: float, pitch_deg: float, yaw_deg: float) -> Quat:
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    return Quat(
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ).normalized()


def quat_to_euler_deg(q: Quat) -> tuple[float, float, float]:
    q = q.normalized()
    x, y, z, w = q.x, q.y, q.z, q.w
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


def quat_slerp(q0: Quat, q1: Quat, t: float) -> Quat:
    t = clamp01(t)
    a = q0.normalized()
    b = q1.normalized()
    dot = a.dot(b)
    if dot < 0.0:
        b = b.negated()
        dot = -dot
    if dot > 0.9995:
        return Quat(
            a.x + (b.x - a.x) * t,
            a.y + (b.y - a.y) * t,
            a.z + (b.z - a.z) * t,
            a.w + (b.w - a.w) * t,
        ).normalized()
    theta0 = math.acos(dot)
    sin_theta0 = math.sin(theta0)
    theta = theta0 * t
    sin_theta = math.sin(theta)
    s0 = math.cos(theta) - dot * sin_theta / sin_theta0
    s1 = sin_theta / sin_theta0
    return Quat(
        s0 * a.x + s1 * b.x,
        s0 * a.y + s1 * b.y,
        s0 * a.z + s1 * b.z,
        s0 * a.w + s1 * b.w,
    ).normalized()


def interpolate_pose(start: Pose, end: Pose, t: float, lock_orientation: bool = False) -> Pose:
    """Linear position interpolation + optional quaternion lock for side-push."""
    t = clamp01(t)
    pos = start.pos.lerp(end.pos, t)
    quat = start.quat if lock_orientation else quat_slerp(start.quat, end.quat, t)
    return Pose(pos=pos, quat=quat)


@dataclass
class SimTray:
    tray_id: str
    dims_m: Vec3
    pose: Pose
    is_clean: bool = False
    gravity_enabled: bool = True
    bound_to_tool: bool = False
    tool_offset: Vec3 = field(default_factory=lambda: Vec3(0.0, 0.0, 0.0))

    def bind_to_tool(self, ee_pose: Pose, side_offset: Vec3) -> None:
        # Simulated fixed joint / child-node parenting.
        self.bound_to_tool = True
        self.gravity_enabled = False
        self.tool_offset = side_offset
        self.sync_with_tool(ee_pose)

    def release_from_tool(self, pose: Pose) -> None:
        self.bound_to_tool = False
        self.gravity_enabled = True
        self.pose = pose.copy()

    def sync_with_tool(self, ee_pose: Pose) -> None:
        if self.bound_to_tool:
            self.pose = Pose(pos=ee_pose.pos + self.tool_offset, quat=ee_pose.quat)


@dataclass
class TrajectorySegment:
    name: str
    start: Pose
    end: Pose
    duration_s: float
    lock_orientation: bool = False
    on_start: Optional[Callable[[], None]] = None
    on_complete: Optional[Callable[[], None]] = None

    def sample(self, t_s: float) -> Pose:
        if self.duration_s <= 1e-6:
            return self.end.copy()
        return interpolate_pose(self.start, self.end, t_s / self.duration_s, self.lock_orientation)


class TrajectorySequence:
    def __init__(self, segments: list[TrajectorySegment]):
        self.segments = segments
        self.index = 0
        self.t = 0.0
        self.finished = len(segments) == 0
        self._started = False

    @property
    def current(self) -> Optional[TrajectorySegment]:
        if self.finished or self.index >= len(self.segments):
            return None
        return self.segments[self.index]

    def step(self, dt: float) -> Pose:
        if self.finished:
            if self.segments:
                return self.segments[-1].end.copy()
            raise RuntimeError("Empty trajectory")
        seg = self.current
        assert seg is not None
        if not self._started:
            self._started = True
            if seg.on_start:
                seg.on_start()
        self.t += max(0.0, dt)
        pose = seg.sample(self.t)
        if self.t >= seg.duration_s:
            if seg.on_complete:
                seg.on_complete()
            self.index += 1
            self.t = 0.0
            self._started = False
            if self.index >= len(self.segments):
                self.finished = True
        return pose


class SidePickState(str, Enum):
    DETECT_DIRTY = "DETECT_DIRTY"
    PICK_DIRTY_SIDE = "PICK_DIRTY_SIDE"
    PLACE_TO_WASHER = "PLACE_TO_WASHER"
    WAIT_AND_DETECT_CLEAN = "WAIT_AND_DETECT_CLEAN"
    PICK_CLEAN_SIDE = "PICK_CLEAN_SIDE"
    PLACE_CLEAN = "PLACE_CLEAN"
    FAULT = "FAULT"


@dataclass
class WorkcellLayout:
    dirty_rack_pick: Vec3 = field(default_factory=lambda: Vec3(-1.50, 1.20, 0.80))
    clean_rack_place_base: Vec3 = field(default_factory=lambda: Vec3(-1.50, -1.20, 0.80))
    washer_infeed: Vec3 = field(default_factory=lambda: Vec3(1.20, 1.00, 0.50))
    return_pick: Vec3 = field(default_factory=lambda: Vec3(1.20, -1.00, 0.50))
    dirty_observe: Vec3 = field(default_factory=lambda: Vec3(-1.80, 0.95, 0.95))
    return_observe: Vec3 = field(default_factory=lambda: Vec3(0.95, -1.30, 0.95))
    safe_home: Vec3 = field(default_factory=lambda: Vec3(0.0, 0.0, 1.0))


@dataclass
class SidePickConfig:
    # Side-pick approach geometry
    tcp_offset_m: float = 0.06  # flange/tool origin to suction contact point
    pregrasp_offset_x_m: float = 0.15
    approach_standoff_m: float = 0.15  # preferred replacement for pregrasp_offset_x_m
    retreat_lift_m: float = 0.02
    contact_dwell_s: float = 0.20
    release_dwell_s: float = 0.15
    motion_speed_mps: float = 0.35
    contact_push_speed_scale: float = 0.80
    retreat_push_speed_scale: float = 0.90
    tray_thickness_m: float = 0.05
    washer_cycle_s: float = 8.0
    post_place_clean_hold_s: float = 3.0
    # Vision mock noise (simulates USB camera + tag solve error)
    vision_noise_xy_m: float = 0.015
    vision_confidence_threshold: float = 0.90


class VisionGuidedSidePickFSM:
    """FSM for vision-guided side pick, washer transfer, and clean-stack placement."""

    HORIZONTAL_SIDE_PICK_EULER_DEG = (0.0, 0.0, 0.0)

    def __init__(
        self,
        robot: Optional[RobotSim] = None,
        vision: Optional[VisionSim] = None,
        vacuum: Optional[VacuumSim] = None,
        layout: Optional[WorkcellLayout] = None,
        cfg: Optional[SidePickConfig] = None,
        ik_solver: Optional[object] = None,
    ):
        self.robot = robot or RobotSim()
        self.vision = vision or VisionSim()
        self.vacuum = vacuum or VacuumSim()
        self.layout = layout or WorkcellLayout()
        self.cfg = cfg or SidePickConfig()
        self.state = SidePickState.DETECT_DIRTY
        self.state_enter_ts = time.time()
        self.total_cycles = 0
        self.clean_stack_count = 0
        self.last_error = ""
        self.sim_time_s = 0.0

        self._traj: Optional[TrajectorySequence] = None
        self._ee_pose = self._pose_from_robot()
        self._side_pick_quat = euler_deg_to_quat(*self.HORIZONTAL_SIDE_PICK_EULER_DEG)
        self._joint_angles_rad = [float(v) for v in self.robot.get_joint_angles_rad()]
        self._ik_solver = ik_solver
        self._ik_init_error = ""
        self._ik_warned = False
        self._current_target: Optional[Vec3] = None
        self._active_tray: Optional[SimTray] = None
        self._washer_tray: Optional[SimTray] = None
        self._washer_elapsed_s = 0.0
        self._last_placed_clean_tray: Optional[SimTray] = None
        self._post_place_clean_hold_until_s = 0.0
        self._dirty_trays = self._make_initial_dirty_trays()
        self._init_ik_solver()
        self._update_joint_solution_from_ee_pose()

    # ----- public loop -----

    def step(self, dt: float) -> None:
        dt = max(0.0, min(dt, 0.1))
        self.sim_time_s += dt
        self.vision.update_scan(dt)

        if self._traj is not None:
            pose = self._traj.step(dt)
            self._apply_ee_pose(pose)
            if self._traj.finished:
                self._traj = None

        if self._active_tray and self._active_tray.bound_to_tool:
            self._active_tray.sync_with_tool(self._ee_pose)

        self._update_washer_tray_sim(dt)

        if self.state == SidePickState.DETECT_DIRTY:
            self._handle_detect_dirty()
        elif self.state == SidePickState.PICK_DIRTY_SIDE:
            self._handle_pick_dirty_side()
        elif self.state == SidePickState.PLACE_TO_WASHER:
            self._handle_place_to_washer()
        elif self.state == SidePickState.WAIT_AND_DETECT_CLEAN:
            self._handle_wait_and_detect_clean()
        elif self.state == SidePickState.PICK_CLEAN_SIDE:
            self._handle_pick_clean_side()
        elif self.state == SidePickState.PLACE_CLEAN:
            self._handle_place_clean()

    def snapshot(self) -> dict:
        roll, pitch, yaw = quat_to_euler_deg(self._ee_pose.quat)
        return {
            "state": self.state.value,
            "robot_pose": {
                "x_m": round(self._ee_pose.pos.x, 4),
                "y_m": round(self._ee_pose.pos.y, 4),
                "z_m": round(self._ee_pose.pos.z, 4),
                "roll_deg": round(roll, 1),
                "pitch_deg": round(pitch, 1),
                "yaw_deg": round(yaw, 1),
            },
            "vacuum_on": self.vacuum.vacuum_on,
            "vacuum_ok": self.vacuum.vacuum_ok if self.vacuum.vacuum_on else False,
            "cycles": self.total_cycles,
            "clean_stack_count": self.clean_stack_count,
            "target": None if self._current_target is None else {
                "x_m": round(self._current_target.x, 4),
                "y_m": round(self._current_target.y, 4),
                "z_m": round(self._current_target.z, 4),
            },
            "active_tray": None if self._active_tray is None else {
                "id": self._active_tray.tray_id,
                "is_clean": self._active_tray.is_clean,
                "bound_to_tool": self._active_tray.bound_to_tool,
                "gravity_enabled": self._active_tray.gravity_enabled,
                "pose": {
                    "x_m": round(self._active_tray.pose.pos.x, 4),
                    "y_m": round(self._active_tray.pose.pos.y, 4),
                    "z_m": round(self._active_tray.pose.pos.z, 4),
                },
            },
            "last_error": self.last_error,
        }

    def get_joint_angles_rad(self) -> list[float]:
        return [round(float(v), 4) for v in self._joint_angles_rad]

    # ----- state handlers -----

    def _handle_detect_dirty(self) -> None:
        if self._traj is None and self._ee_pose.pos.distance(self.layout.dirty_observe) > 0.02:
            observe = self._make_pose(self.layout.dirty_observe, self._side_pick_quat)
            self._start_sequence([self._linear_segment("move_dirty_observe", self._ee_pose, observe, True)])
            return
        if self._traj is not None:
            return
        target = self._detect_top_dirty_tray_side()
        if target is None:
            return
        self._current_target = target
        self._transition(SidePickState.PICK_DIRTY_SIDE)

    def _handle_pick_dirty_side(self) -> None:
        if self._traj is None and self._active_tray is None:
            tray = self._pop_dirty_tray()
            if tray is None:
                self._fault("Dirty tray rack empty")
                return
            self._active_tray = tray
            side_target = self._current_target or self._tray_side_center_for_pick(tray)
            self._start_sequence(self._build_side_pick_segments(side_target, tray))
            return
        if self._traj is None and self._active_tray and self._active_tray.bound_to_tool:
            self._transition(SidePickState.PLACE_TO_WASHER)
            return
        if self._traj is None and self._active_tray and not self._active_tray.bound_to_tool:
            self._fault("Side pick failed: vacuum seal not established")

    def _handle_place_to_washer(self) -> None:
        if self._active_tray is None:
            if self._washer_tray is not None:
                self._transition(SidePickState.WAIT_AND_DETECT_CLEAN)
                return
            self._fault("No tray to place into washer line")
            return
        if self._traj is None and self._washer_tray is None:
            self._start_sequence(self._build_place_to_washer_segments(self._active_tray))
            return

    def _handle_wait_and_detect_clean(self) -> None:
        if self._traj is None and self._ee_pose.pos.distance(self.layout.return_observe) > 0.02:
            observe = self._make_pose(self.layout.return_observe, self._side_pick_quat)
            self._start_sequence([self._linear_segment("move_return_observe", self._ee_pose, observe, True)])
            return
        if self._traj is not None:
            return
        target = self._detect_clean_tray_at_return()
        if target is None:
            return
        self._current_target = target
        self._transition(SidePickState.PICK_CLEAN_SIDE)

    def _handle_pick_clean_side(self) -> None:
        if self._tray_ready_for_clean_pick() is False and self._active_tray is None:
            self._fault("No clean tray available on return conveyor")
            return
        if self._traj is None and self._active_tray is None:
            self._active_tray = self._washer_tray
            self._washer_tray = None
            side_target = self._current_target or self._tray_side_center_for_pick(self._active_tray)
            self._start_sequence(self._build_side_pick_segments(side_target, self._active_tray))
            return
        if self._traj is None and self._active_tray and self._active_tray.bound_to_tool:
            self._transition(SidePickState.PLACE_CLEAN)
            return
        if self._traj is None and self._active_tray and not self._active_tray.bound_to_tool:
            self._fault("Clean tray side pick failed: vacuum seal not established")

    def _handle_place_clean(self) -> None:
        if self._active_tray is None:
            if self._traj is None:
                if self._post_place_clean_hold_until_s > self.sim_time_s:
                    return
                self._last_placed_clean_tray = None
                self._post_place_clean_hold_until_s = 0.0
                self.total_cycles += 1
                self._transition(SidePickState.DETECT_DIRTY)
            return
        if self._traj is None:
            self._start_sequence(self._build_place_clean_segments(self._active_tray))
            return

    # ----- trajectory builders -----

    def _build_side_pick_segments(self, target: Vec3, tray: SimTray) -> list[TrajectorySegment]:
        # Reset IK seed per planned sequence to reduce branch drift across cycles.
        if self._ik_solver is not None and hasattr(self._ik_solver, "reset_seed"):
            self._ik_solver.reset_seed()
        # `target` is the detected tray side-center (not tray center) in world coordinates.
        q = self._side_pick_quat
        approach_sign = self._approach_sign_for_target(target)
        standoff = self._approach_standoff_m()
        contact = Vec3(target.x + approach_sign * self.cfg.tcp_offset_m, target.y, target.z)
        pre = Vec3(contact.x + approach_sign * standoff, contact.y, contact.z)
        lift = Vec3(contact.x, contact.y, contact.z + self.cfg.retreat_lift_m)
        retreat = Vec3(pre.x, pre.y, lift.z)

        start_pose = self._ee_pose.copy()
        pre_pose = self._solve_ik_with_locked_orientation(pre, q)
        contact_pose = self._solve_ik_with_locked_orientation(contact, q)
        lift_pose = self._solve_ik_with_locked_orientation(lift, q)
        retreat_pose = self._solve_ik_with_locked_orientation(retreat, q)

        # tray_center = ee_flange + tool_offset while suction is attached
        tray_side_offset = Vec3(-approach_sign * (tray.dims_m.x / 2.0 + self.cfg.tcp_offset_m), 0.0, 0.0)

        def on_vacuum_on() -> None:
            self.vacuum.turn_on()
            if self.vacuum.vacuum_ok:
                tray.bind_to_tool(self._ee_pose, tray_side_offset)

        return [
            self._linear_segment("pre_grasp", start_pose, pre_pose, lock_orientation=True),
            self._horizontal_push_segment("contact_push", pre_pose, contact_pose, speed_scale=self.cfg.contact_push_speed_scale),
            TrajectorySegment(
                name="vacuum_on_dwell",
                start=contact_pose,
                end=contact_pose,
                duration_s=self.cfg.contact_dwell_s,
                lock_orientation=True,
                on_start=on_vacuum_on,
            ),
            self._linear_segment("retreat_lift_2cm", contact_pose, lift_pose, lock_orientation=True),
            self._horizontal_push_segment("retreat_back_15cm", lift_pose, retreat_pose, speed_scale=self.cfg.retreat_push_speed_scale),
        ]

    def _build_place_to_washer_segments(self, tray: SimTray) -> list[TrajectorySegment]:
        # Reset IK seed per planned sequence to reduce branch drift across cycles.
        if self._ik_solver is not None and hasattr(self._ik_solver, "reset_seed"):
            self._ik_solver.reset_seed()
        q = self._side_pick_quat
        infeed = self.layout.washer_infeed
        target_tray_center = Vec3(infeed.x, infeed.y, infeed.z)
        approach_sign = self._approach_sign_for_target(target_tray_center)
        contact = Vec3(
            target_tray_center.x + approach_sign * (tray.dims_m.x / 2.0 + self.cfg.tcp_offset_m),
            target_tray_center.y,
            target_tray_center.z,
        )
        pre = Vec3(contact.x + approach_sign * 0.20, contact.y, contact.z)
        retreat = Vec3(contact.x + approach_sign * 0.20, contact.y, contact.z + 0.03)
        home = self.layout.safe_home

        start_pose = self._ee_pose.copy()
        pre_pose = self._solve_ik_with_locked_orientation(pre, q)
        place_pose = self._solve_ik_with_locked_orientation(contact, q)
        retreat_pose = self._solve_ik_with_locked_orientation(retreat, q)
        home_pose = self._solve_ik_with_locked_orientation(home, q)

        def on_release() -> None:
            self.vacuum.turn_off()
            tray.is_clean = False
            # Release at the exact infeed center so washer path takeover starts from
            # the same coordinate (prevents visible handoff "teleport").
            exact_infeed_pose = self._make_pose(target_tray_center, self._side_pick_quat)
            tray.release_from_tool(exact_infeed_pose)
            self._washer_tray = tray
            self._active_tray = None
            self._washer_elapsed_s = 0.0

        return [
            self._linear_segment("washer_approach", start_pose, pre_pose, lock_orientation=True),
            self._horizontal_push_segment("washer_insert", pre_pose, place_pose, speed_scale=self.cfg.contact_push_speed_scale),
            TrajectorySegment(
                name="vacuum_off_release",
                start=place_pose,
                end=place_pose,
                duration_s=self.cfg.release_dwell_s,
                lock_orientation=True,
                on_start=on_release,
            ),
            self._horizontal_push_segment("washer_retreat", place_pose, retreat_pose, speed_scale=self.cfg.retreat_push_speed_scale),
            self._linear_segment("washer_clear", retreat_pose, home_pose, lock_orientation=True),
        ]

    def _build_place_clean_segments(self, tray: SimTray) -> list[TrajectorySegment]:
        # Reset IK seed per planned sequence to reduce branch drift across cycles.
        if self._ik_solver is not None and hasattr(self._ik_solver, "reset_seed"):
            self._ik_solver.reset_seed()
        q = self._side_pick_quat
        base = self.layout.clean_rack_place_base
        place_z = base.z + self.clean_stack_count * self.cfg.tray_thickness_m
        target_tray_center = Vec3(base.x, base.y, place_z)
        approach_sign = self._approach_sign_for_target(target_tray_center)
        place = Vec3(
            target_tray_center.x + approach_sign * (tray.dims_m.x / 2.0 + self.cfg.tcp_offset_m),
            target_tray_center.y,
            target_tray_center.z,
        )
        pre = Vec3(place.x + approach_sign * 0.15, place.y, place.z)
        retreat = Vec3(place.x + approach_sign * 0.20, place.y, place.z + 0.03)
        home = self.layout.safe_home

        start_pose = self._ee_pose.copy()
        pre_pose = self._solve_ik_with_locked_orientation(pre, q)
        place_pose = self._solve_ik_with_locked_orientation(place, q)
        retreat_pose = self._solve_ik_with_locked_orientation(retreat, q)
        home_pose = self._solve_ik_with_locked_orientation(home, q)

        def on_release() -> None:
            self.vacuum.turn_off()
            tray.is_clean = True
            exact_place_pose = self._make_pose(target_tray_center, self._side_pick_quat)
            tray.release_from_tool(exact_place_pose)
            self.clean_stack_count += 1
            self._last_placed_clean_tray = tray
            self._post_place_clean_hold_until_s = self.sim_time_s + max(0.0, self.cfg.post_place_clean_hold_s)
            self._active_tray = None

        return [
            self._linear_segment("clean_pre", start_pose, pre_pose, lock_orientation=True),
            self._horizontal_push_segment("clean_contact", pre_pose, place_pose, speed_scale=self.cfg.contact_push_speed_scale),
            TrajectorySegment(
                name="clean_release",
                start=place_pose,
                end=place_pose,
                duration_s=self.cfg.release_dwell_s,
                lock_orientation=True,
                on_start=on_release,
            ),
            self._horizontal_push_segment("clean_retreat", place_pose, retreat_pose, speed_scale=self.cfg.retreat_push_speed_scale),
            self._linear_segment("clean_home", retreat_pose, home_pose, lock_orientation=True),
        ]

    def _linear_segment(
        self,
        name: str,
        start: Pose,
        end: Pose,
        lock_orientation: bool = False,
        speed_mps: Optional[float] = None,
    ) -> TrajectorySegment:
        dist = start.pos.distance(end.pos)
        speed = max(0.05, speed_mps or self.cfg.motion_speed_mps)
        duration = max(0.05, dist / speed)
        return TrajectorySegment(name, start, end, duration, lock_orientation=lock_orientation)

    def _horizontal_push_segment(self, name: str, start: Pose, end: Pose, speed_scale: float = 0.8) -> TrajectorySegment:
        """Straight horizontal push with orientation lock (no roll/pitch/yaw drift)."""
        end_h = end.copy()
        end_h.pos.z = start.pos.z  # lock vertical height during side-contact push
        return self._linear_segment(
            name,
            start,
            end_h,
            lock_orientation=True,
            speed_mps=self.cfg.motion_speed_mps * max(0.2, speed_scale),
        )

    def _start_sequence(self, segments: list[TrajectorySegment]) -> None:
        self._traj = TrajectorySequence(segments)

    # ----- vision / washer simulation -----

    def _detect_top_dirty_tray_side(self) -> Optional[Vec3]:
        tray = self._peek_dirty_tray()
        if tray is None:
            return None
        true_side = self._tray_side_center_for_pick(tray)
        self.vision.default_target.x = true_side.x
        self.vision.default_target.y = true_side.y
        self.vision.default_target.z = true_side.z
        d = self.vision.detect()
        if d.get("detected") and float(d.get("confidence", 0.0)) >= self.cfg.vision_confidence_threshold:
            return self._mock_vision_detect(true_side)
        return None

    def _detect_clean_tray_at_return(self) -> Optional[Vec3]:
        tray = self._washer_tray
        if tray is None or not tray.is_clean:
            return None
        if tray.pose.pos.distance(self.layout.return_pick) > 0.03:
            return None
        true_side = self._tray_side_center_for_pick(tray)
        self.vision.default_target.x = true_side.x
        self.vision.default_target.y = true_side.y
        self.vision.default_target.z = true_side.z
        d = self.vision.detect()
        if d.get("detected") and float(d.get("confidence", 0.0)) >= self.cfg.vision_confidence_threshold:
            return self._mock_vision_detect(true_side)
        return None

    def _update_washer_tray_sim(self, dt: float) -> None:
        tray = self._washer_tray
        if tray is None:
            return
        # Sample at current elapsed time first, then advance elapsed.
        # This keeps the first takeover frame exactly at washer_infeed and avoids
        # a visible jump when vacuum is released.
        cycle_s = max(0.001, self.cfg.washer_cycle_s)
        progress = clamp01(self._washer_elapsed_s / cycle_s)

        # Simulated U-shaped conveyor path: infeed -> right arc -> return_pick.
        arc_center_x = 2.30
        arc_center_y = 0.0
        arc_radius = 1.10
        z = self.layout.washer_infeed.z
        s = self.layout.washer_infeed
        e = self.layout.return_pick

        if progress < 0.25:
            t = progress / 0.25
            pos = Vec3(s.x + (arc_center_x - s.x) * t, s.y + (arc_center_y + arc_radius - s.y) * t, z)
        elif progress < 0.75:
            t = (progress - 0.25) / 0.5
            theta = math.pi / 2.0 - math.pi * t
            pos = Vec3(
                arc_center_x + arc_radius * math.cos(theta),
                arc_center_y + arc_radius * math.sin(theta),
                z,
            )
        else:
            t = (progress - 0.75) / 0.25
            arc_exit = Vec3(arc_center_x - arc_radius, arc_center_y - arc_radius, z)
            pos = Vec3(arc_exit.x + (e.x - arc_exit.x) * t, arc_exit.y + (e.y - arc_exit.y) * t, z)

        tray.pose = Pose(pos=pos, quat=self._side_pick_quat)
        self._washer_elapsed_s = min(cycle_s, self._washer_elapsed_s + max(0.0, dt))
        if self._washer_elapsed_s >= cycle_s:
            tray.is_clean = True

    # ----- helpers -----

    def _pose_from_robot(self) -> Pose:
        return Pose(
            pos=Vec3(self.robot.x, self.robot.y, self.robot.z),
            quat=euler_deg_to_quat(
                float(getattr(self.robot, "roll", 0.0)),
                float(getattr(self.robot, "pitch", 0.0)),
                float(getattr(self.robot, "yaw", 0.0)),
            ),
        )

    def _apply_ee_pose(self, pose: Pose) -> None:
        self._ee_pose = pose
        self.robot.x = pose.pos.x
        self.robot.y = pose.pos.y
        self.robot.z = pose.pos.z
        roll, pitch, yaw = quat_to_euler_deg(pose.quat)
        self.robot.roll = roll
        self.robot.pitch = pitch
        self.robot.yaw = yaw
        self._update_joint_solution_from_ee_pose()

    def _init_ik_solver(self) -> None:
        if self._ik_solver is not None:
            return
        try:
            self._ik_solver = CRX20IkPySolver(find_default_crx20_urdf_path())
        except Exception as exc:
            self._ik_solver = None
            self._ik_init_error = str(exc)

    def _update_joint_solution_from_ee_pose(self) -> None:
        solver = self._ik_solver
        if solver is None:
            if self._ik_init_error and not self._ik_warned:
                print(f"[IK] Disabled, using placeholder joint angles: {self._ik_init_error}")
                self._ik_warned = True
            self._joint_angles_rad = [float(v) for v in self.robot.get_joint_angles_rad()]
            return

        try:
            res: IKResult = solver.solve_tcp_pose(
                [self._ee_pose.pos.x, self._ee_pose.pos.y, self._ee_pose.pos.z],
                [self._ee_pose.quat.x, self._ee_pose.quat.y, self._ee_pose.quat.z, self._ee_pose.quat.w],
            )
        except Exception as exc:
            if not self._ik_warned:
                print(f"[IK] Solver runtime failure, keeping previous solution: {exc}")
                self._ik_warned = True
            return

        if not res.ok or len(res.joints_rad) != 6:
            if res.error and not self._ik_warned:
                print(f"[IK] Solver returned invalid solution, keeping previous solution: {res.error}")
                self._ik_warned = True
            return

        # The FSM trajectory is already time-interpolated in Cartesian space.
        # Applying an extra low-pass filter in joint space introduces visual lag,
        # causing "remote suction" and release teleport artifacts in the frontend.
        self._joint_angles_rad = [float(v) for v in res.joints_rad]

    def _make_pose(self, pos: Vec3, quat: Quat) -> Pose:
        return Pose(pos.copy(), quat)

    def _approach_standoff_m(self) -> float:
        return float(self.cfg.approach_standoff_m or self.cfg.pregrasp_offset_x_m)

    def _approach_sign_for_target(self, target: Vec3) -> float:
        # Approach from the current TCP side to avoid crossing through the tray.
        return 1.0 if self._ee_pose.pos.x >= target.x else -1.0

    def _tray_side_center_for_pick(self, tray: SimTray) -> Vec3:
        sign = self._approach_sign_for_target(tray.pose.pos)
        return Vec3(
            tray.pose.pos.x + sign * (tray.dims_m.x / 2.0),
            tray.pose.pos.y,
            tray.pose.pos.z,
        )

    def _mock_vision_detect(self, true_pos: Vec3) -> Vec3:
        n = max(0.0, float(self.cfg.vision_noise_xy_m))
        return Vec3(
            true_pos.x + random.uniform(-n, n),
            true_pos.y + random.uniform(-n, n),
            true_pos.z,
        )

    def _solve_ik_with_locked_orientation(self, pos: Vec3, locked_quat: Quat) -> Pose:
        """IK placeholder for current sim platform.

        Real integration point: call IK with a hard orientation constraint, then
        convert the solved TCP pose to joint targets. In this simulation we use
        the Cartesian pose directly but keep the quaternion attached so the
        trajectory interpolator can lock orientation.
        """
        return Pose(pos.copy(), locked_quat)

    def _transition(self, state: SidePickState) -> None:
        self.state = state
        self.state_enter_ts = time.time()
        self._traj = None
        self.vision.start_scan()

    def _fault(self, msg: str) -> None:
        self.last_error = msg
        self.state = SidePickState.FAULT
        self._traj = None

    def _peek_dirty_tray(self) -> Optional[SimTray]:
        return self._dirty_trays[-1] if self._dirty_trays else None

    def _pop_dirty_tray(self) -> Optional[SimTray]:
        return self._dirty_trays.pop() if self._dirty_trays else None

    def _tray_ready_for_clean_pick(self) -> bool:
        tray = self._washer_tray
        return bool(tray is not None and tray.is_clean and tray.pose.pos.distance(self.layout.return_pick) <= 0.03)

    def _make_initial_dirty_trays(self) -> list[SimTray]:
        trays: list[SimTray] = []
        base = self.layout.dirty_rack_pick
        for i in range(4):
            trays.append(
                SimTray(
                    tray_id=f"dirty_{i+1}",
                    dims_m=Vec3(0.40, 0.40, self.cfg.tray_thickness_m),
                    pose=Pose(
                        pos=Vec3(base.x, base.y, base.z + i * self.cfg.tray_thickness_m),
                        quat=self._side_pick_quat,
                    ),
                    is_clean=False,
                )
            )
        return trays


def run_demo(seconds: float = 20.0, dt: float = 0.02) -> None:
    """Console demo of the FSM without touching the existing FastAPI app."""
    fsm = VisionGuidedSidePickFSM()
    start = time.time()
    next_print = 0.0
    while time.time() - start < seconds and fsm.state != SidePickState.FAULT:
        fsm.step(dt)
        elapsed = time.time() - start
        if elapsed >= next_print:
            s = fsm.snapshot()
            print(
                f"{elapsed:5.1f}s {s['state']:<22} "
                f"pos=({s['robot_pose']['x_m']:+.2f},{s['robot_pose']['y_m']:+.2f},{s['robot_pose']['z_m']:+.2f}) "
                f"vac={'ON' if s['vacuum_on'] else 'OFF'} cycles={s['cycles']}"
            )
            next_print += 0.5
        time.sleep(dt)
    if fsm.state == SidePickState.FAULT:
        print("FAULT:", fsm.last_error)


if __name__ == "__main__":
    run_demo()
