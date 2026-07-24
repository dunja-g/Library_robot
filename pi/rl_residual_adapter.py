"""Raspberry Pi adapter for the Library Robot Minimal Residual SAC controller."""

from __future__ import annotations

import logging
import os
import sys
import time
from typing import Any, Optional

logger = logging.getLogger(__name__)

# Add RL repository root to path if available relative to workspace
_RL_REPO_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..", "cc+hacker final")
)
if os.path.isdir(_RL_REPO_DIR) and _RL_REPO_DIR not in sys.path:
    sys.path.insert(0, _RL_REPO_DIR)

try:
    from library_residual import (
        LibraryObservationV1,
        ResidualResult,
        SafeResidualPolicy,
    )
    _RL_PACKAGE_AVAILABLE = True
except ImportError:
    _RL_PACKAGE_AVAILABLE = False
    LibraryObservationV1 = None
    SafeResidualPolicy = None

    class ResidualResult:  # type: ignore
        def __init__(self, **kwargs):
            self.normalized_action = kwargs.get("normalized_action", 0.0)
            self.residual_pwm = kwargs.get("residual_pwm", 0)
            self.valid = kwargs.get("valid", False)
            self.apply_to_motor = kwargs.get("apply_to_motor", False)
            self.latency_ms = kwargs.get("latency_ms", 0.0)
            self.reason = kwargs.get("reason", "rl_package_missing")


class RLResidualAdapter:
    """Adapter bridging GridController and SafeResidualPolicy.

    Loads configuration from environment variables:
      • LIBRARY_ROBOT_RL_MODE: disabled | shadow | active (default: disabled)
      • LIBRARY_ROBOT_RL_MODEL_DIR: path to exported bundle
      • LIBRARY_ROBOT_RL_MAX_RESIDUAL_PWM: max correction (default: 10)
      • LIBRARY_ROBOT_RL_DEADLINE_MS: max allowed inference latency (default: 50)
      • LIBRARY_ROBOT_RL_TELEMETRY_AGE_MS: max observation age (default: 250)
    """

    def __init__(
        self,
        mode: str = "disabled",
        model_dir: Optional[str] = None,
        max_residual_pwm: int = 10,
        deadline_ms: float = 50.0,
        observation_max_age_ms: float = 250.0,
    ):
        self.mode = mode.lower().strip()
        self.model_dir = model_dir or os.getenv(
            "LIBRARY_ROBOT_RL_MODEL_DIR",
            os.path.join(
                os.path.dirname(__file__), "..", "..", "..", "cc+hacker final", "models", "library_sac"
            ),
        )
        self.max_residual_pwm = max_residual_pwm
        self.deadline_ms = deadline_ms
        self.observation_max_age_ms = observation_max_age_ms
        self.policy = None

        if not _RL_PACKAGE_AVAILABLE:
            logger.warning(
                "library_residual package is not installed; RL adapter disabled"
            )
            self.mode = "disabled"
            return

        if self.mode != "disabled":
            try:
                self.policy = SafeResidualPolicy.load(
                    model_directory=self.model_dir,
                    mode=self.mode,
                    max_residual_pwm=self.max_residual_pwm,
                    deadline_ms=self.deadline_ms,
                    observation_max_age_ms=self.observation_max_age_ms,
                )
            except Exception as exc:
                logger.error("Failed to load SafeResidualPolicy: %s", exc)
                self.mode = "disabled"

    @classmethod
    def from_env(cls) -> "RLResidualAdapter":
        """Factory initializing settings from environment variables."""
        mode = os.getenv("LIBRARY_ROBOT_RL_MODE", "disabled")
        model_dir = os.getenv("LIBRARY_ROBOT_RL_MODEL_DIR")
        max_pwm = int(os.getenv("LIBRARY_ROBOT_RL_MAX_RESIDUAL_PWM", "10"))
        deadline = float(os.getenv("LIBRARY_ROBOT_RL_DEADLINE_MS", "50.0"))
        max_age = float(os.getenv("LIBRARY_ROBOT_RL_TELEMETRY_AGE_MS", "250.0"))
        return cls(
            mode=mode,
            model_dir=model_dir,
            max_residual_pwm=max_pwm,
            deadline_ms=deadline,
            observation_max_age_ms=max_age,
        )

    def step(
        self,
        *,
        is_forward: bool,
        completed_distance: float,
        target_distance: float,
        target_heading_deg: float,
        fused_heading_deg: float,
        left_distance_cm: float,
        right_distance_cm: float,
        front_ultrasonic_cm: float,
        current_action: str = "FORWARD",
    ) -> ResidualResult:
        """Construct observation and run inference if enabled."""
        if self.mode == "disabled" or self.policy is None:
            return ResidualResult(reason="disabled")

        obs = LibraryObservationV1.from_navigation_state(
            is_forward=is_forward,
            completed_distance=completed_distance,
            target_distance=target_distance,
            target_heading_deg=target_heading_deg,
            fused_heading_deg=fused_heading_deg,
            left_distance_cm=left_distance_cm,
            right_distance_cm=right_distance_cm,
            front_ultrasonic_cm=front_ultrasonic_cm,
        )

        return self.policy.predict(
            observation=obs,
            current_action=current_action,
            timestamp=time.monotonic(),
        )

    def get_status(self) -> dict:
        """Return diagnostic status for dashboard/monitoring."""
        loaded = (
            self.policy.model_loaded
            if self.policy is not None
            else False
        )
        return {
            "mode": self.mode,
            "package_available": _RL_PACKAGE_AVAILABLE,
            "model_loaded": loaded,
            "model_dir": self.model_dir,
            "max_residual_pwm": self.max_residual_pwm,
        }
