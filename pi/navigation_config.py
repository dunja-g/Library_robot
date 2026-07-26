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
    last_row_return_align_tolerance_px: int = 90
    stop_distance_cm: float = 35.0
    obstacle_distance_cm: float = 20.0
    scan_timeout_seconds: float = 60.0
    target_confirmation_frames: int = 2
    alignment_confirmation_frames: int = 2
    target_loss_tolerance_frames: int = 3
    min_marker_area_px: float = 0.0
    aruco_enhance_vision: bool = True
    aruco_clahe_clip_limit: float = 3.0
    aruco_clahe_tile_grid: int = 8
    aruco_target_area_px: float = 8000.0
    aruco_align_pulse_seconds: float = 0.2
    aruco_align_settle_seconds: float = 2.0
    aruco_align_fine_pulse_seconds: float = 0.12
    aruco_align_fine_settle_seconds: float = 0.4
    aruco_align_max_search_pulses: int = 6
    aruco_align_max_reacquire_pulses: int = 2
    aruco_align_invert_turn: bool = False
    aruco_steering_kp: float = -0.15
    aruco_track_candidates: bool = True
    aruco_upscale_factor: float = 2.0
    aruco_candidate_min_area_px: float = 900.0
    aruco_candidate_max_area_px: float = 120000.0
    aruco_candidate_confirmation_frames: int = 2
    aruco_candidate_max_jump_px: float = 80.0
    return_obstacle_distance_cm: float = 10.0
    invert_turn_direction: bool = False
    turn_90_seconds: float = 0.8
    uturn_seconds: float = 1.6
    destination_dwell_seconds: float = 5.0
    auto_return: bool = True
    rl_mode: str = "disabled"
    rl_model_dir: str = ""
    rl_max_bias: int = 5
    rl_deadline_ms: float = 50.0
    rl_invert_bias: bool = False

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
        if self.last_row_return_align_tolerance_px < 0:
            raise ValueError(
                "last_row_return_align_tolerance_px must be non-negative"
            )
        if self.target_loss_tolerance_frames < 0:
            raise ValueError("target_loss_tolerance_frames must be non-negative")
        if self.min_marker_area_px < 0:
            raise ValueError("min_marker_area_px must be non-negative")
        if self.aruco_clahe_clip_limit <= 0:
            raise ValueError("aruco_clahe_clip_limit must be positive")
        if self.aruco_clahe_tile_grid <= 0:
            raise ValueError("aruco_clahe_tile_grid must be positive")
        if self.aruco_target_area_px <= 0:
            raise ValueError("aruco_target_area_px must be positive")
        if self.aruco_align_pulse_seconds <= 0:
            raise ValueError("aruco_align_pulse_seconds must be positive")
        if self.aruco_align_settle_seconds <= 0:
            raise ValueError("aruco_align_settle_seconds must be positive")
        if self.aruco_align_fine_pulse_seconds <= 0:
            raise ValueError("aruco_align_fine_pulse_seconds must be positive")
        if self.aruco_align_fine_settle_seconds <= 0:
            raise ValueError("aruco_align_fine_settle_seconds must be positive")
        if self.aruco_align_max_search_pulses <= 0:
            raise ValueError("aruco_align_max_search_pulses must be positive")
        if self.aruco_align_max_reacquire_pulses < 0:
            raise ValueError(
                "aruco_align_max_reacquire_pulses must be non-negative"
            )
        if self.aruco_upscale_factor < 1.0:
            raise ValueError("aruco_upscale_factor must be at least 1.0")
        if self.aruco_candidate_min_area_px < 0:
            raise ValueError("aruco_candidate_min_area_px must be non-negative")
        if self.aruco_candidate_max_area_px <= self.aruco_candidate_min_area_px:
            raise ValueError(
                "aruco_candidate_max_area_px must exceed aruco_candidate_min_area_px"
            )
        if self.aruco_candidate_confirmation_frames < 2:
            raise ValueError(
                "aruco_candidate_confirmation_frames must be at least 2"
            )
        if self.aruco_candidate_max_jump_px <= 0:
            raise ValueError("aruco_candidate_max_jump_px must be positive")
        if self.return_obstacle_distance_cm <= 0:
            raise ValueError("return_obstacle_distance_cm must be positive")
        if self.destination_dwell_seconds < 0:
            raise ValueError("destination_dwell_seconds must be non-negative")
        if self.rl_mode not in {"disabled", "shadow", "active"}:
            raise ValueError("rl_mode must be 'disabled', 'shadow', or 'active'")
        if self.rl_max_bias < 0:
            raise ValueError("rl_max_bias must be non-negative")
        if self.rl_deadline_ms <= 0:
            raise ValueError("rl_deadline_ms must be positive")

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
            last_row_return_align_tolerance_px=_env_number(
                "LIBRARY_ROBOT_LAST_ROW_RETURN_ALIGN_TOLERANCE_PX", 90, int
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
            aruco_enhance_vision=_env_bool(
                "LIBRARY_ROBOT_ARUCO_ENHANCE_VISION", True
            ),
            aruco_clahe_clip_limit=_env_number(
                "LIBRARY_ROBOT_ARUCO_CLAHE_CLIP_LIMIT", 3.0, float
            ),
            aruco_clahe_tile_grid=_env_number(
                "LIBRARY_ROBOT_ARUCO_CLAHE_TILE_GRID", 8, int
            ),
            aruco_target_area_px=_env_number(
                "LIBRARY_ROBOT_ARUCO_TARGET_AREA_PX", 8000.0, float
            ),
            aruco_align_pulse_seconds=_env_number(
                "LIBRARY_ROBOT_ARUCO_ALIGN_PULSE_SECONDS", 0.2, float
            ),
            aruco_align_settle_seconds=_env_number(
                "LIBRARY_ROBOT_ARUCO_ALIGN_SETTLE_SECONDS", 2.0, float
            ),
            aruco_align_fine_pulse_seconds=_env_number(
                "LIBRARY_ROBOT_ARUCO_ALIGN_FINE_PULSE_SECONDS", 0.12, float
            ),
            aruco_align_fine_settle_seconds=_env_number(
                "LIBRARY_ROBOT_ARUCO_ALIGN_FINE_SETTLE_SECONDS", 0.4, float
            ),
            aruco_align_max_search_pulses=_env_number(
                "LIBRARY_ROBOT_ARUCO_ALIGN_MAX_SEARCH_PULSES", 6, int
            ),
            aruco_align_max_reacquire_pulses=_env_number(
                "LIBRARY_ROBOT_ARUCO_ALIGN_MAX_REACQUIRE_PULSES", 2, int
            ),
            aruco_align_invert_turn=_env_bool(
                "LIBRARY_ROBOT_ARUCO_ALIGN_INVERT_TURN", False
            ),
            aruco_steering_kp=_env_number(
                "LIBRARY_ROBOT_ARUCO_STEERING_KP", -0.15, float
            ),
            aruco_track_candidates=_env_bool(
                "LIBRARY_ROBOT_ARUCO_TRACK_CANDIDATES", True
            ),
            aruco_upscale_factor=_env_number(
                "LIBRARY_ROBOT_ARUCO_UPSCALE_FACTOR", 2.0, float
            ),
            aruco_candidate_min_area_px=_env_number(
                "LIBRARY_ROBOT_ARUCO_CANDIDATE_MIN_AREA_PX", 900.0, float
            ),
            aruco_candidate_max_area_px=_env_number(
                "LIBRARY_ROBOT_ARUCO_CANDIDATE_MAX_AREA_PX", 120000.0, float
            ),
            aruco_candidate_confirmation_frames=_env_number(
                "LIBRARY_ROBOT_ARUCO_CANDIDATE_CONFIRMATION_FRAMES", 2, int
            ),
            aruco_candidate_max_jump_px=_env_number(
                "LIBRARY_ROBOT_ARUCO_CANDIDATE_MAX_JUMP_PX", 80.0, float
            ),
            return_obstacle_distance_cm=_env_number(
                "LIBRARY_ROBOT_RETURN_OBSTACLE_DISTANCE_CM", 10.0, float
            ),
            invert_turn_direction=_env_bool(
                "LIBRARY_ROBOT_INVERT_TURN_DIRECTION", False
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
            rl_mode=os.getenv("LIBRARY_ROBOT_RL_MODE", "disabled").strip().lower(),
            rl_model_dir=os.getenv("LIBRARY_ROBOT_RL_MODEL_DIR", "").strip(),
            rl_max_bias=_env_number("LIBRARY_ROBOT_RL_MAX_BIAS", 5, int),
            rl_deadline_ms=_env_number(
                "LIBRARY_ROBOT_RL_DEADLINE_MS", 50.0, float
            ),
            rl_invert_bias=_env_bool(
                "LIBRARY_ROBOT_RL_INVERT_BIAS", False
            ),
        )
