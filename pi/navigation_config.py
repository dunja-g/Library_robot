"""Runtime configuration for camera capture and fixed-grid navigation."""

from __future__ import annotations

import os
from dataclasses import dataclass


def _env_number(name: str, default, cast):
    raw = os.getenv(name)
    if raw is None:
        return default
    try:
        return cast(raw)
    except ValueError as exc:
        raise ValueError(f"{name} must be a valid {cast.__name__}") from exc


def _env_bool(name: str, default: bool) -> bool:
    raw = os.getenv(name)
    if raw is None:
        return default
    normalized = raw.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"{name} must be a valid boolean")


@dataclass(frozen=True)
class NavigationConfig:
    camera_width: int = 640
    camera_height: int = 480
    camera_fps: int = 20
    control_hz: float = 10.0
    align_tolerance_px: int = 30
    stop_distance_cm: float = 35.0
    obstacle_distance_cm: float = 20.0
    scan_timeout_seconds: float = 60.0
    target_confirmation_frames: int = 2
    alignment_confirmation_frames: int = 2
    target_loss_tolerance_frames: int = 3
    min_marker_area_px: float = 0.0
    turn_90_seconds: float = 0.8
    uturn_seconds: float = 1.6
    destination_dwell_seconds: float = 5.0
    auto_return: bool = True

    def __post_init__(self):
        positive = {
            "camera_width": self.camera_width,
            "camera_height": self.camera_height,
            "camera_fps": self.camera_fps,
            "control_hz": self.control_hz,
            "stop_distance_cm": self.stop_distance_cm,
            "obstacle_distance_cm": self.obstacle_distance_cm,
            "scan_timeout_seconds": self.scan_timeout_seconds,
            "target_confirmation_frames": self.target_confirmation_frames,
            "alignment_confirmation_frames": self.alignment_confirmation_frames,
            "turn_90_seconds": self.turn_90_seconds,
            "uturn_seconds": self.uturn_seconds,
        }
        invalid = [name for name, value in positive.items() if value <= 0]
        if invalid:
            raise ValueError(f"Configuration values must be positive: {invalid}")
        if self.align_tolerance_px < 0:
            raise ValueError("align_tolerance_px must be non-negative")
        if self.target_loss_tolerance_frames < 0:
            raise ValueError("target_loss_tolerance_frames must be non-negative")
        if self.min_marker_area_px < 0:
            raise ValueError("min_marker_area_px must be non-negative")
        if self.destination_dwell_seconds < 0:
            raise ValueError("destination_dwell_seconds must be non-negative")

    @classmethod
    def from_env(cls) -> "NavigationConfig":
        """Load settings from ``LIBRARY_ROBOT_*`` environment variables."""
        return cls(
            camera_width=_env_number("LIBRARY_ROBOT_CAMERA_WIDTH", 640, int),
            camera_height=_env_number("LIBRARY_ROBOT_CAMERA_HEIGHT", 480, int),
            camera_fps=_env_number("LIBRARY_ROBOT_CAMERA_FPS", 20, int),
            control_hz=_env_number("LIBRARY_ROBOT_CONTROL_HZ", 10.0, float),
            align_tolerance_px=_env_number(
                "LIBRARY_ROBOT_ALIGN_TOLERANCE_PX", 30, int
            ),
            stop_distance_cm=_env_number(
                "LIBRARY_ROBOT_STOP_DISTANCE_CM", 35.0, float
            ),
            obstacle_distance_cm=_env_number(
                "LIBRARY_ROBOT_OBSTACLE_DISTANCE_CM", 20.0, float
            ),
            scan_timeout_seconds=_env_number(
                "LIBRARY_ROBOT_SCAN_TIMEOUT_SECONDS", 60.0, float
            ),
            target_confirmation_frames=_env_number(
                "LIBRARY_ROBOT_TARGET_CONFIRMATION_FRAMES", 2, int
            ),
            alignment_confirmation_frames=_env_number(
                "LIBRARY_ROBOT_ALIGNMENT_CONFIRMATION_FRAMES", 2, int
            ),
            target_loss_tolerance_frames=_env_number(
                "LIBRARY_ROBOT_TARGET_LOSS_TOLERANCE_FRAMES", 3, int
            ),
            min_marker_area_px=_env_number(
                "LIBRARY_ROBOT_MIN_MARKER_AREA_PX", 0.0, float
            ),
            turn_90_seconds=_env_number(
                "LIBRARY_ROBOT_TURN_90_SECONDS", 0.8, float
            ),
            uturn_seconds=_env_number(
                "LIBRARY_ROBOT_UTURN_SECONDS", 1.6, float
            ),
            destination_dwell_seconds=_env_number(
                "LIBRARY_ROBOT_DESTINATION_DWELL_SECONDS", 5.0, float
            ),
            auto_return=_env_bool("LIBRARY_ROBOT_AUTO_RETURN", True),
        )
