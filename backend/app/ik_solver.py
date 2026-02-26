"""Lightweight IK solver integration for CRX-20 URDF using ikpy.

This module is intentionally backend-only and independent from Three.js. It
solves joint angles from the FSM's Cartesian TCP pose so the frontend URDF can
be driven by kinematically consistent joint values instead of placeholder
trigonometric waveforms.
"""
from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

try:
    import numpy as np
except Exception:  # pragma: no cover - optional until dependency is installed
    np = None  # type: ignore[assignment]

try:
    from ikpy.chain import Chain
except Exception:  # pragma: no cover - optional until dependency is installed
    Chain = None  # type: ignore[assignment]


CRX20_JOINT_NAMES = ("J1", "J2", "J3", "J4", "J5", "J6")


@dataclass
class IKResult:
    ok: bool
    joints_rad: list[float]
    error: str = ""


def find_default_crx20_urdf_path() -> Path:
    """Find the CRX-20 URDF in both local-dev and Docker container layouts."""
    env_path = os.getenv("CRX20_IK_URDF_PATH", "").strip()
    candidates: list[Path] = []
    if env_path:
        candidates.append(Path(env_path))

    here = Path(__file__).resolve()
    # Local repo layout: d:/repo/backend/app/*.py -> d:/repo/frontend/assets/...
    candidates.append(here.parents[2] / "frontend" / "assets" / "robot" / "crx20ial" / "robot.urdf")
    # Backend Docker image layout: /app/app/*.py + frontend copied to /app/static
    candidates.append(here.parents[1] / "static" / "assets" / "robot" / "crx20ial" / "robot.urdf")
    # Current working directory fallbacks
    candidates.append(Path.cwd() / "frontend" / "assets" / "robot" / "crx20ial" / "robot.urdf")
    candidates.append(Path.cwd() / "static" / "assets" / "robot" / "crx20ial" / "robot.urdf")

    for path in candidates:
        if path.is_file():
            return path
    tried = "\n".join(str(p) for p in candidates)
    raise FileNotFoundError(f"CRX-20 URDF not found. Tried:\n{tried}")


class CRX20IkPySolver:
    """ikpy-backed numerical IK solver for the CRX-20 frontend URDF."""

    def __init__(self, urdf_path: str | Path):
        if Chain is None or np is None:
            raise RuntimeError("ikpy/numpy not installed")

        self.urdf_path = Path(urdf_path)
        if not self.urdf_path.is_file():
            raise FileNotFoundError(f"URDF not found: {self.urdf_path}")

        self._chain = Chain.from_urdf_file(str(self.urdf_path))
        self._joint_indices = self._build_joint_index_map()
        self._last_solution = np.zeros(len(self._chain.links), dtype=float)

    def _build_joint_index_map(self) -> list[int]:
        names = [str(getattr(link, "name", "")) for link in self._chain.links]
        index_map: list[int] = []
        for wanted in CRX20_JOINT_NAMES:
            if wanted in names:
                index_map.append(names.index(wanted))
                continue
            # Small fallback for odd naming variants
            found = next((i for i, n in enumerate(names) if n.strip().lower() == wanted.lower()), None)
            if found is None:
                raise RuntimeError(f"IK chain missing joint '{wanted}'. Links={names}")
            index_map.append(found)
        return index_map

    def reset_seed(self) -> None:
        """Reset initial position to zero, preventing cumulative twist between FSM cycles."""
        self._last_solution = np.zeros(len(self._chain.links), dtype=float)

    def solve_tcp_pose(
        self,
        xyz_m: Sequence[float],
        side_pick_quat_xyzw: Sequence[float] | None = None,  # reserved for orientation-constrained solve
    ) -> IKResult:
        del side_pick_quat_xyzw  # orientation constraint can be added in a later refinement

        try:
            target_position = np.array([float(xyz_m[0]), float(xyz_m[1]), float(xyz_m[2])], dtype=float)
            solution = self._chain.inverse_kinematics(
                target_position=target_position,
                initial_position=self._last_solution,
            )
            full = np.asarray(solution, dtype=float)
            if full.shape[0] != len(self._chain.links):
                return IKResult(False, [0.0] * 6, "Unexpected ikpy solution length")
            self._last_solution = full
            joints = [float(full[idx]) for idx in self._joint_indices]
            return IKResult(True, joints)
        except Exception as exc:
            return IKResult(False, [0.0] * 6, f"ikpy solve failed: {exc}")
