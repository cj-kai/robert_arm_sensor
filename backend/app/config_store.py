"""Simple JSON-backed runtime configuration store for the execution service."""
from __future__ import annotations

import json
from copy import deepcopy
from pathlib import Path
from typing import Any, Dict


DEFAULT_CONFIG: Dict[str, Any] = {
    "camera": {
        "mount": "wrist_usb",
        "working_distance_m": 0.40,
        "working_distance_range_m": [0.30, 0.50],
        "view_angle_deg": 25,
        "stream_resolution": [1280, 720],
        "fps": 30,
    },
    "vision": {
        "detector": "aruco",
        "tag_dictionary": "DICT_4X4_50",
        "confidence_threshold": 0.85,
        "filter_window": 5,
        "exposure": None,
        "gain": None,
    },
    "tray_profiles": {
        "square_tray_10_75": {
            "label": "Square Tray 10.75",
            "dims_m": {"x_m": 0.2731, "y_m": 0.2731, "z_m": 0.0318},
            "tag_size_m": 0.05,
        },
        "rect_tray_11_125x7_75": {
            "label": "Rect Tray 11.125x7.75",
            "dims_m": {"x_m": 0.2826, "y_m": 0.1969, "z_m": 0.0191},
            "tag_size_m": 0.05,
        },
        "bin_16_625x11x5": {
            "label": "Bin 16.625x11x5",
            "dims_m": {"x_m": 0.4223, "y_m": 0.2794, "z_m": 0.1270},
            "tag_size_m": 0.07,
        },
    },
    "grasp": {
        "pregrasp_z_m": 0.25,
        "contact_z_m": 0.02,
        "lift_z_m": 0.20,
        "vacuum_wait_timeout_s": 0.3,
    },
    "motion": {
        "speed_scale": 0.3,
        "accel_scale": 0.3,
        "move_timeout_s": 5.0,
    },
    "recovery": {
        "search_dwell_s": 0.25,
        "max_search_rounds": 2,
        "search_points": [
            {"x_m": -1.50, "y_m": 1.20, "z_m": 0.70},
            {"x_m": -1.44, "y_m": 1.12, "z_m": 0.70},
            {"x_m": -1.56, "y_m": 1.28, "z_m": 0.70},
        ],
    },
    "conveyor": {
        "start_settle_s": 0.2,
        "run_timeout_s": 10.0,
        "stop_buffer_s": 0.1,
    },
    "washer": {
        "model": "MIMASA DA-80",
        "clearance_m": {"width_m": 0.45, "height_m": 0.35},
        "cycle_time_s": 12.0,
        "mode": "simulated",
    },
    "stations": {
        "rack_dirty_pick": {"x_m": -1.50, "y_m": 1.20, "z_m": 0.80},
        "rack_clean_place": {"x_m": -1.50, "y_m": -1.20, "z_m": 0.80},
        "washer_load": {"x_m": 1.20, "y_m": 1.00, "z_m": 0.50},
        "washer_return_pick": {"x_m": 1.20, "y_m": -1.00, "z_m": 0.50},
        "conveyor_place": {"x_m": 1.20, "y_m": 1.00, "z_m": 0.50},
        "conveyor_pick": {"x_m": 1.20, "y_m": -1.00, "z_m": 0.50},
    },
}


def _deep_merge(base: Dict[str, Any], patch: Dict[str, Any]) -> Dict[str, Any]:
    out = deepcopy(base)
    for key, value in patch.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _deep_merge(out[key], value)
        else:
            out[key] = value
    return out


class ConfigStore:
    def __init__(self, path: Path | None = None):
        self.path = path or (Path(__file__).parent.parent / "data" / "config.json")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._cache: Dict[str, Any] = self._load_or_init()

    def _load_or_init(self) -> Dict[str, Any]:
        if not self.path.exists():
            self._write(DEFAULT_CONFIG)
            return deepcopy(DEFAULT_CONFIG)
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
            if not isinstance(data, dict):
                raise ValueError("config.json root must be an object")
            return _deep_merge(DEFAULT_CONFIG, data)
        except Exception:
            # If the file is broken, keep the service usable and restore defaults.
            self._write(DEFAULT_CONFIG)
            return deepcopy(DEFAULT_CONFIG)

    def _write(self, data: Dict[str, Any]) -> None:
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
        tmp.replace(self.path)

    def get(self) -> Dict[str, Any]:
        return deepcopy(self._cache)

    def update(self, patch: Dict[str, Any]) -> Dict[str, Any]:
        if not isinstance(patch, dict):
            raise ValueError("config patch must be an object")
        self._cache = _deep_merge(self._cache, patch)
        self._write(self._cache)
        return self.get()
