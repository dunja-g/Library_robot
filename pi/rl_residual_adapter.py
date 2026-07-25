"""Shadow-safe bridge from navigation telemetry to a trained residual policy.

The adapter is deliberately self-contained: it loads a TorchScript actor plus
its manifest and normalization sidecars straight from disk, so it does not
depend on the training repository being installed on the robot. Every failure
path degrades to "suggest nothing" rather than raising, because this runs
inside the control loop.
"""

from __future__ import annotations

import json
import logging
import math
import os
import time
from typing import Any, Callable, Iterable

logger = logging.getLogger(__name__)

MODES = ("disabled", "shadow", "active")

# Used only when the manifest does not declare an observation layout. Anything
# produced under this layout is reported as unverified because a mismatch with
# the training layout silently feeds the policy garbage.
DEFAULT_OBSERVATION_FIELDS = (
    "heading_error_deg",
    "progress_ratio",
    "front_distance_cm",
    "left_distance_cm",
    "right_distance_cm",
)

_MANIFEST_FIELD_KEYS = ("observation_fields", "obs_fields", "fields")


def _coerce_float(value: Any) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return 0.0
    return number if math.isfinite(number) else 0.0


class RLResidualAdapter:
    """Turn grid-controller telemetry into a bounded steering-bias suggestion.

    ``shadow`` mode runs the full inference and safety pipeline but always
    reports ``applied_bias == 0``, which makes it safe to enable on real
    hardware while the policy is still being evaluated.
    """

    def __init__(
        self,
        *,
        mode: str = "disabled",
        model_dir: str | None = None,
        max_bias: int = 5,
        deadline_ms: float = 50.0,
        serial_bridge: Any | None = None,
        infer: Callable[[list[float]], float] | None = None,
        clock: Callable[[], float] = time.monotonic,
    ):
        normalized_mode = str(mode).strip().lower()
        if normalized_mode not in MODES:
            raise ValueError(f"mode must be one of {MODES}")
        if max_bias < 0:
            raise ValueError("max_bias must be non-negative")
        if deadline_ms <= 0:
            raise ValueError("deadline_ms must be positive")

        self.requested_mode = normalized_mode
        self.mode = normalized_mode
        self.model_dir = model_dir
        self.max_bias = int(max_bias)
        self.deadline_ms = float(deadline_ms)
        self.serial = serial_bridge
        self._clock = clock

        self._infer = infer
        self._backend = "injected" if infer is not None else None
        self.observation_fields: tuple[str, ...] = DEFAULT_OBSERVATION_FIELDS
        self.observation_spec = "assumed_default"
        self.action_scale = 1.0
        self._mean: list[float] = []
        self._std: list[float] = []
        self.load_error: str | None = None
        self._last_result = self._inactive_result("not_run")

        if self.mode == "disabled":
            return
        if self._infer is None:
            self._load_bundle()
        if self._infer is None:
            self.mode = "disabled"

    @classmethod
    def from_config(
        cls,
        config: Any,
        *,
        serial_bridge: Any | None = None,
        infer: Callable[[list[float]], float] | None = None,
    ) -> "RLResidualAdapter":
        return cls(
            mode=getattr(config, "rl_mode", "disabled"),
            model_dir=getattr(config, "rl_model_dir", "") or None,
            max_bias=getattr(config, "rl_max_bias", 5),
            deadline_ms=getattr(config, "rl_deadline_ms", 50.0),
            serial_bridge=serial_bridge,
            infer=infer,
        )

    # ---------------------------------------------------------------- loading

    def _load_bundle(self) -> None:
        if not self.model_dir:
            self.load_error = "model_dir_not_set"
            return
        if not os.path.isdir(self.model_dir):
            self.load_error = "model_dir_missing"
            return

        self._load_manifest()
        self._load_normalization()

        actor_path = os.path.join(self.model_dir, "actor.ts")
        if not os.path.isfile(actor_path):
            self.load_error = "actor_ts_missing"
            return
        try:
            import torch
        except ImportError:
            self.load_error = "torch_not_installed"
            return

        try:
            module = torch.jit.load(actor_path, map_location="cpu")
            module.eval()
        except Exception as exc:
            self.load_error = f"actor_load_failed:{type(exc).__name__}"
            logger.error("Failed to load TorchScript actor: %s", exc)
            return

        def _infer(vector: list[float]) -> float:
            with torch.no_grad():
                tensor = torch.tensor([vector], dtype=torch.float32)
                output = module(tensor)
            if isinstance(output, (tuple, list)):
                output = output[0]
            return float(output.detach().reshape(-1)[0].item())

        self._infer = _infer
        self._backend = "torchscript"
        self.load_error = None

    def _load_manifest(self) -> None:
        manifest = self._read_json("manifest.json")
        if not isinstance(manifest, dict):
            return

        observation = manifest.get("observation")
        sources: Iterable[Any] = (
            [manifest.get(key) for key in _MANIFEST_FIELD_KEYS]
            + ([observation.get(key) for key in _MANIFEST_FIELD_KEYS]
               if isinstance(observation, dict) else [])
        )
        for candidate in sources:
            if isinstance(candidate, list) and candidate:
                self.observation_fields = tuple(str(item) for item in candidate)
                self.observation_spec = "manifest"
                break

        scale = manifest.get("action_scale")
        if scale is not None:
            scale_value = _coerce_float(scale)
            if scale_value > 0:
                self.action_scale = scale_value

    def _load_normalization(self) -> None:
        payload = self._read_json("normalization.json")
        if not isinstance(payload, dict):
            return
        self._mean = self._normalization_vector(payload.get("mean"), 0.0)
        self._std = self._normalization_vector(payload.get("std"), 1.0)

    def _normalization_vector(self, raw: Any, default: float) -> list[float]:
        if isinstance(raw, dict):
            return [
                _coerce_float(raw.get(name, default))
                for name in self.observation_fields
            ]
        if isinstance(raw, list) and len(raw) == len(self.observation_fields):
            return [_coerce_float(item) for item in raw]
        return []

    def _read_json(self, filename: str) -> Any:
        path = os.path.join(self.model_dir or "", filename)
        if not os.path.isfile(path):
            return None
        try:
            with open(path, "r", encoding="utf-8") as handle:
                return json.load(handle)
        except (OSError, ValueError) as exc:
            logger.warning("Could not read %s: %s", filename, exc)
            return None

    # ------------------------------------------------------------ observation

    @staticmethod
    def build_features(status: dict) -> dict[str, float]:
        """Flatten a ``GridController.get_status()`` payload into named features."""
        telemetry = status.get("telemetry") or {}
        ultrasonic = telemetry.get("ultrasonic") or {}
        imu = telemetry.get("imu") or {}
        encoders = telemetry.get("encoders") or {}

        action = status.get("current_action") or ""
        target_ticks = _coerce_float(status.get("target_ticks"))
        completed = min(
            abs(_coerce_float(encoders.get("left"))),
            abs(_coerce_float(encoders.get("right"))),
        )
        progress_ratio = completed / target_ticks if target_ticks > 0 else 0.0

        fused = _coerce_float(imu.get("heading_fused_deg"))
        encoder_heading = _coerce_float(imu.get("heading_encoder_deg"))

        left = _coerce_float(ultrasonic.get("left"))
        right = _coerce_float(ultrasonic.get("right"))

        return {
            "heading_error_deg": fused - encoder_heading,
            "fused_heading_deg": fused,
            "encoder_heading_deg": encoder_heading,
            "progress_ratio": max(0.0, min(1.0, progress_ratio)),
            "remaining_ticks": max(0.0, target_ticks - completed),
            "front_distance_cm": _coerce_float(ultrasonic.get("center")),
            "left_distance_cm": left,
            "right_distance_cm": right,
            "lateral_offset_cm": left - right,
            "speed_correction": _coerce_float(imu.get("speed_correction")),
            "is_forward": 1.0 if action == "FORWARD" else 0.0,
            "is_backward": 1.0 if action == "BACKWARD" else 0.0,
        }

    def _vectorize(self, features: dict[str, float]) -> list[float]:
        vector = [
            _coerce_float(features.get(name, 0.0))
            for name in self.observation_fields
        ]
        if len(self._mean) == len(vector):
            vector = [value - mean for value, mean in zip(vector, self._mean)]
        if len(self._std) == len(vector):
            vector = [
                value / std if std else value
                for value, std in zip(vector, self._std)
            ]
        return vector

    # ------------------------------------------------------------------- step

    def step_from_status(self, status: dict) -> dict:
        """Run one inference against controller telemetry and gate the result."""
        result = self._evaluate(status)
        self._last_result = result
        return result

    def _evaluate(self, status: dict) -> dict:
        if self.mode == "disabled" or self._infer is None:
            return self._inactive_result(self.load_error or "disabled")

        action = status.get("current_action") or ""
        if action not in {"FORWARD", "BACKWARD"}:
            return self._inactive_result("not_linear_motion")
        if status.get("state") not in {"MOVING", "RETURNING"}:
            return self._inactive_result("not_moving")

        ultrasonic = (status.get("telemetry") or {}).get("ultrasonic") or {}
        if ultrasonic.get("status") != "OK":
            return self._inactive_result("telemetry_unavailable")

        features = self.build_features(status)
        vector = self._vectorize(features)

        started = self._clock()
        try:
            raw_output = self._infer(vector)
        except Exception as exc:
            logger.warning("RL inference failed: %s", exc)
            return self._inactive_result(f"inference_failed:{type(exc).__name__}")
        inference_ms = (self._clock() - started) * 1000.0

        if not math.isfinite(raw_output):
            return self._inactive_result("invalid_output", inference_ms=inference_ms)

        clamped_output = max(-1.0, min(1.0, raw_output * self.action_scale))
        suggested = int(round(clamped_output * self.max_bias))
        suggested = max(-self.max_bias, min(self.max_bias, suggested))

        if inference_ms > self.deadline_ms:
            return self._result(
                model_output=raw_output,
                suggested_bias=suggested,
                applied_bias=0,
                reason="deadline_exceeded",
                inference_ms=inference_ms,
            )

        if self.mode == "shadow":
            return self._result(
                model_output=raw_output,
                suggested_bias=suggested,
                applied_bias=0,
                reason="shadow_mode",
                inference_ms=inference_ms,
            )

        applied = self._apply_bias(suggested)
        return self._result(
            model_output=raw_output,
            suggested_bias=suggested,
            applied_bias=applied,
            reason="applied" if applied == suggested else "apply_failed",
            inference_ms=inference_ms,
        )

    def _apply_bias(self, bias: int) -> int:
        setter = getattr(self.serial, "set_steer_bias", None)
        if not callable(setter):
            return 0
        try:
            return bias if setter(bias) else 0
        except Exception as exc:
            logger.warning("Failed to send steer bias: %s", exc)
            return 0

    # ---------------------------------------------------------------- results

    def _result(
        self,
        *,
        model_output: float | None,
        suggested_bias: int,
        applied_bias: int,
        reason: str,
        inference_ms: float,
    ) -> dict:
        return {
            "enabled": self.mode != "disabled",
            "mode": self.mode,
            "model_output": None if model_output is None else round(model_output, 4),
            "suggested_bias": suggested_bias,
            "applied_bias": applied_bias,
            "reason": reason,
            "inference_ms": round(inference_ms, 2),
        }

    def _inactive_result(self, reason: str, *, inference_ms: float = 0.0) -> dict:
        return self._result(
            model_output=None,
            suggested_bias=0,
            applied_bias=0,
            reason=reason,
            inference_ms=inference_ms,
        )

    def get_status(self) -> dict:
        status = dict(self._last_result)
        status.update(
            requested_mode=self.requested_mode,
            backend=self._backend,
            model_dir=self.model_dir,
            model_loaded=self._infer is not None,
            load_error=self.load_error,
            max_bias=self.max_bias,
            deadline_ms=self.deadline_ms,
            observation_spec=self.observation_spec,
            observation_fields=list(self.observation_fields),
        )
        return status
