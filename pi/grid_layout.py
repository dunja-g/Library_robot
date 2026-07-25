"""Parameterised 2-column by 3-row box layout and encoder/timed route planning."""

from __future__ import annotations

import os
import math
from dataclasses import dataclass


BOX_IDS = tuple(f"{row}{column}" for row in range(1, 4) for column in ("A", "B"))
MOTION_ACTIONS = {
    "FORWARD",
    "BACKWARD",
    "TURN_LEFT",
    "TURN_RIGHT",
    "UTURN",
    "ARUCO_ALIGN",
    "ARUCO_GUIDED_FORWARD",
    "ARUCO_APPROACH",
}

# Printable markers from aruco_codes/generate_markers.py (DICT_5X5_50, IDs 0–4).
HALLWAY_MARKER_ID = 0
ROW_MARKER_IDS = {1: 1, 2: 2, 3: 3}
# Per-box markers from data/generate_aruco_shelf_markers.py (IDs 1–6 on shelf fronts).
BOX_MARKER_IDS = {
    "1A": 6,
    "1B": 1,
    "2A": 2,
    "2B": 3,
    "3A": 4,
    "3B": 5,
}
GRID_MARKERS = {
    HALLWAY_MARKER_ID: "Main hallway",
    1: "Row 1 aisle / box 1B",
    2: "Box 2A",
    3: "Box 2B",
    4: "Box 3A",
    5: "Box 3B",
    6: "Box 1A",
}


def normalize_box_id(box_id: str) -> str:
    normalized = str(box_id).strip().upper()
    if normalized not in BOX_IDS:
        raise ValueError(f"box_id must be one of: {', '.join(BOX_IDS)}")
    return normalized


def _optional_env_float(name: str) -> float | None:
    raw = os.getenv(name)
    if raw is None or not raw.strip():
        return None
    try:
        value = float(raw)
    except ValueError as exc:
        raise ValueError(f"{name} must be a number") from exc
    if value <= 0:
        raise ValueError(f"{name} must be positive")
    return value


@dataclass(frozen=True)
class GridGeometry:
    """Physical dimensions. ``None`` means measurement is still pending."""

    first_row_distance_cm: float | None = None
    row_spacing_cm: float | None = None
    box_approach_distance_cm: float | None = None
    forward_speed_cms: float | None = None  # cm/s for timed driving (no encoder)
    outbound_turn_degrees: float | None = None
    return_turn_degrees: float | None = None

    def __post_init__(self):
        for name, value in (
            ("first_row_distance_cm", self.first_row_distance_cm),
            ("row_spacing_cm", self.row_spacing_cm),
            ("box_approach_distance_cm", self.box_approach_distance_cm),
            ("forward_speed_cms", self.forward_speed_cms),
            ("outbound_turn_degrees", self.outbound_turn_degrees),
            ("return_turn_degrees", self.return_turn_degrees),
        ):
            if value is not None and value <= 0:
                raise ValueError(f"{name} must be positive")

    @classmethod
    def from_env(cls) -> "GridGeometry":
        return cls(
            first_row_distance_cm=_optional_env_float(
                "LIBRARY_ROBOT_GRID_FIRST_ROW_CM"
            ),
            row_spacing_cm=_optional_env_float(
                "LIBRARY_ROBOT_GRID_ROW_SPACING_CM"
            ),
            box_approach_distance_cm=_optional_env_float(
                "LIBRARY_ROBOT_GRID_APPROACH_CM"
            ),
            forward_speed_cms=_optional_env_float(
                "LIBRARY_ROBOT_FORWARD_SPEED_CMS"
            ),
        )

    @property
    def missing_fields(self) -> list[str]:
        return [
            name
            for name, value in (
                ("first_row_distance_cm", self.first_row_distance_cm),
                ("row_spacing_cm", self.row_spacing_cm),
                ("box_approach_distance_cm", self.box_approach_distance_cm),
            )
            if value is None
        ]

    def distance_to_row(self, row: int) -> float:
        if self.missing_fields:
            raise ValueError(
                "Grid dimensions are not configured: " + ", ".join(self.missing_fields)
            )
        if row not in range(1, 4):
            raise ValueError("row must be between 1 and 3")
        return float(self.first_row_distance_cm) + (row - 1) * float(
            self.row_spacing_cm
        )


@dataclass(frozen=True)
class EncoderCalibration:
    ticks_per_cm: float | None = None
    turn_90_ticks: int | None = None
    turn_180_ticks: int | None = None
    ticks_per_revolution: float | None = None
    wheel_diameter_cm: float | None = None

    def __post_init__(self):
        for name, value in (
            ("ticks_per_cm", self.ticks_per_cm),
            ("turn_90_ticks", self.turn_90_ticks),
            ("turn_180_ticks", self.turn_180_ticks),
            ("ticks_per_revolution", self.ticks_per_revolution),
            ("wheel_diameter_cm", self.wheel_diameter_cm),
        ):
            if value is not None and value <= 0:
                raise ValueError(f"{name} must be positive")

    @classmethod
    def from_env(cls) -> "EncoderCalibration":
        ticks_per_cm = _optional_env_float("LIBRARY_ROBOT_ENCODER_TICKS_PER_CM")
        turn_90 = _optional_env_float("LIBRARY_ROBOT_ENCODER_TURN_90_TICKS")
        turn_180 = _optional_env_float("LIBRARY_ROBOT_ENCODER_TURN_180_TICKS")
        ticks_per_revolution = _optional_env_float(
            "LIBRARY_ROBOT_ENCODER_TICKS_PER_REV"
        )
        wheel_diameter_cm = _optional_env_float(
            "LIBRARY_ROBOT_WHEEL_DIAMETER_CM"
        )
        return cls(
            ticks_per_cm=ticks_per_cm,
            turn_90_ticks=None if turn_90 is None else int(turn_90),
            turn_180_ticks=None if turn_180 is None else int(turn_180),
            ticks_per_revolution=(
                4.0 if ticks_per_revolution is None else ticks_per_revolution
            ),
            wheel_diameter_cm=(
                6.5 if wheel_diameter_cm is None else wheel_diameter_cm
            ),
        )

    @property
    def missing_fields(self) -> list[str]:
        return self.missing_fields_for("encoder")

    def missing_fields_for(self, turn_source: str) -> list[str]:
        if turn_source not in {"encoder", "imu"}:
            raise ValueError("turn_source must be 'encoder' or 'imu'")
        missing = [
            name
            for name, value in (
                ("turn_90_ticks", self.turn_90_ticks),
                ("turn_180_ticks", self.turn_180_ticks),
            )
            if value is None and turn_source == "encoder"
        ]
        if self.ticks_per_cm is None:
            if self.ticks_per_revolution is None:
                missing.append("ticks_per_revolution")
            if self.wheel_diameter_cm is None:
                missing.append("wheel_diameter_cm")
        return missing

    @property
    def resolved_ticks_per_cm(self) -> float:
        if self.ticks_per_cm is not None:
            return float(self.ticks_per_cm)
        if self.ticks_per_revolution is None or self.wheel_diameter_cm is None:
            raise ValueError("Encoder wheel calibration is not configured")
        return float(self.ticks_per_revolution) / (
            math.pi * float(self.wheel_diameter_cm)
        )

    def distance_ticks(self, distance_cm: float) -> int:
        return max(1, round(distance_cm * self.resolved_ticks_per_cm))


def row_marker_id(row: int) -> int:
    if row not in ROW_MARKER_IDS:
        raise ValueError("row must be between 1 and 3")
    return ROW_MARKER_IDS[row]


def box_marker_id(box_id: str) -> int:
    normalized = normalize_box_id(box_id)
    if normalized not in BOX_MARKER_IDS:
        raise ValueError(f"No ArUco marker mapped for box {normalized}")
    return BOX_MARKER_IDS[normalized]


def approach_marker_id(box_id: str) -> int:
    """Marker used to align and approach a specific box."""
    return box_marker_id(box_id)


def _marker_step(action: str, label: str, marker_id: int, **extra) -> dict:
    step = {
        "action": action,
        "target_aruco_id": marker_id,
        "target_ticks": 0,
        "target_seconds": 0.0,
        "label": label,
    }
    step.update(extra)
    return step


def _timed_step(
    action: str,
    label: str,
    distance_cm: float | None,
    speed_cms: float,
) -> dict:
    """Build a route step driven by time rather than encoder ticks."""
    if distance_cm is not None:
        target_seconds = round(distance_cm / speed_cms, 3)
    else:
        target_seconds = 0.0  # turns handled by IMU; duration is irrelevant
    return {"action": action, "target_ticks": 0, "target_seconds": target_seconds, "label": label}


def build_grid_route(
    box_id: str,
    geometry: GridGeometry,
    calibration: EncoderCalibration,
    turn_source: str = "encoder",
    linear_source: str = "encoder",
    vision_source: str = "aruco",
    return_backout_extra_cm: float = 0.0,
) -> dict:
    """Build a Dock-to-box route and blind encoder reverse return.

    When ``vision_source='aruco'`` the outbound path uses hallway marker 0 and
    row markers 1/2/3 from ``aruco_codes/generate_markers.py``. Return remains
    encoder-only reverse. Select ``vision_source='encoder'`` for legacy routes.
    """
    box_id = normalize_box_id(box_id)

    if linear_source not in {"encoder", "timed"}:
        raise ValueError("linear_source must be 'encoder' or 'timed'")
    if vision_source not in {"encoder", "aruco"}:
        raise ValueError("vision_source must be 'encoder' or 'aruco'")
    timed_mode = linear_source == "timed"
    aruco_mode = vision_source == "aruco"
    if return_backout_extra_cm < 0:
        raise ValueError("return_backout_extra_cm must be non-negative")

    if timed_mode:
        if geometry.forward_speed_cms is None:
            raise ValueError(
                "Timed navigation requires LIBRARY_ROBOT_FORWARD_SPEED_CMS"
            )
        if geometry.missing_fields:
            raise ValueError(
                "Grid navigation is not calibrated: "
                + ", ".join(geometry.missing_fields)
            )
    else:
        missing = geometry.missing_fields + calibration.missing_fields_for(turn_source)
        if missing:
            raise ValueError("Grid navigation is not calibrated: " + ", ".join(missing))

    row = int(box_id[0])
    side = box_id[1]
    outward_turn = "TURN_LEFT" if side == "A" else "TURN_RIGHT"
    return_turn = "TURN_RIGHT" if side == "A" else "TURN_LEFT"
    reverse_turn = "TURN_RIGHT" if side == "A" else "TURN_LEFT"

    if timed_mode:
        speed = float(geometry.forward_speed_cms)
        aisle_cm = geometry.distance_to_row(row)
        approach_cm = float(geometry.box_approach_distance_cm)
        hallway_step = _timed_step("FORWARD", f"Dock to row {row}", aisle_cm, speed)
        turn_step = _timed_step(outward_turn, f"Face box {box_id}", None, speed)
        turn_step["target_degrees"] = geometry.outbound_turn_degrees
        approach_step = _timed_step("FORWARD", f"Approach box {box_id}", approach_cm, speed)

        return_route = [
            _timed_step("BACKWARD", "Back out to centre aisle", approach_cm, speed),
            _timed_step(reverse_turn, "Face original direction", None, speed),
            _timed_step("BACKWARD", "Reverse to Dock", aisle_cm, speed),
        ]
        return_route[1]["target_degrees"] = geometry.return_turn_degrees
    else:
        aisle_ticks = calibration.distance_ticks(geometry.distance_to_row(row))
        approach_ticks = calibration.distance_ticks(
            float(geometry.box_approach_distance_cm)
        )
        hallway_step = {
            "action": "FORWARD",
            "target_ticks": aisle_ticks,
            "target_seconds": 0.0,
            "label": f"Dock to row {row}",
        }
        turn_step = {
            "action": outward_turn,
            "target_ticks": int(calibration.turn_90_ticks or 0),
            "target_seconds": 0.0,
            "label": f"Face box {box_id}",
        }
        approach_step = {
            "action": "FORWARD",
            "target_ticks": approach_ticks,
            "target_seconds": 0.0,
            "label": f"Approach box {box_id}",
        }
        return_route = [
            {
                "action": "BACKWARD",
                "target_ticks": approach_ticks,
                "target_seconds": 0.0,
                "label": "Back out to centre aisle",
            },
            {
                "action": reverse_turn,
                "target_ticks": int(calibration.turn_90_ticks or 0),
                "target_seconds": 0.0,
                "label": "Face original direction",
            },
            {
                "action": "BACKWARD",
                "target_ticks": aisle_ticks,
                "target_seconds": 0.0,
                "label": "Reverse to Dock",
            },
        ]

    if return_backout_extra_cm > 0:
        if timed_mode:
            return_route[0]["target_seconds"] = round(
                float(return_route[0]["target_seconds"])
                + return_backout_extra_cm / speed,
                3,
            )
        else:
            return_route[0]["target_ticks"] = int(return_route[0]["target_ticks"]) + (
                calibration.distance_ticks(return_backout_extra_cm)
            )

    if aruco_mode:
        shelf_marker = approach_marker_id(box_id)
        hallway = dict(hallway_step)
        hallway["track_aruco_id"] = HALLWAY_MARKER_ID
        return_escape = dict(return_route[0])
        return_escape["track_aruco_id"] = shelf_marker
        return_dock = dict(return_route[2])
        return_dock["track_aruco_id"] = HALLWAY_MARKER_ID
        return_route[0] = return_escape
        return_route[2] = return_dock
        outbound = [
            _marker_step(
                "ARUCO_ALIGN",
                "Align on hallway marker",
                HALLWAY_MARKER_ID,
            ),
            hallway,
            turn_step,
            _marker_step(
                "ARUCO_ALIGN",
                f"Align on {box_id} marker",
                shelf_marker,
            ),
            _marker_step(
                "ARUCO_APPROACH",
                f"Approach shelf at {box_id}",
                shelf_marker,
            ),
        ]
    else:
        outbound = [hallway_step, turn_step, approach_step]

    return {
        "box_id": box_id,
        "row": row,
        "column": side,
        "outbound": outbound,
        "return": return_route,
    }
