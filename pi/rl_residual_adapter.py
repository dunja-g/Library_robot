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
import threading
import time
from copy import deepcopy
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

_MANIFEST_FIELD_KEYS = (
    "feature_names",
    "observation_fields",
    "obs_fields",
    "fields",
)

_SUPPORTED_FIELDS = {
    "motion_direction",
    "segment_progress",
    "fused_heading_error",
    "left_right_encoder_error",
    "front_ultrasonic_distance",
    *DEFAULT_OBSERVATION_FIELDS,
    "fused_heading_deg",
    "encoder_heading_deg",
    "remaining_ticks",
    "lateral_offset_cm",
    "speed_correction",
    "is_forward",
    "is_backward",
}


def _coerce_float(value: Any) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return 0.0
    return number if math.isfinite(number) else 0.0


def _finite_float(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _wrap_degrees(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


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
        obstacle_distance_cm: float = 20.0,
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
        if obstacle_distance_cm <= 0:
            raise ValueError("obstacle_distance_cm must be positive")

        self.requested_mode = normalized_mode
        self.mode = normalized_mode
        self.model_dir = model_dir
        self.max_bias = int(max_bias)
        self.deadline_ms = float(deadline_ms)
        self.obstacle_distance_cm = float(obstacle_distance_cm)
        self.serial = serial_bridge
        self._clock = clock

        self._infer = infer
        self._backend = "injected" if infer is not None else None
        self.observation_fields: tuple[str, ...] = DEFAULT_OBSERVATION_FIELDS
        self.observation_spec = "assumed_default"
        self.action_scale = 1.0
        self._feature_clip: dict[str, tuple[float, float]] = {}
        self._normalization_ranges: dict[str, tuple[float, float]] = {}
        self._mean: list[float] = []
        self._std: list[float] = []
        self.load_error: str | None = None
        self._last_result = self._inactive_result("not_run")
        self._state_lock = threading.RLock()
        self._worker_event = threading.Event()
        self._worker_stop = threading.Event()
        self._worker: threading.Thread | None = None
        self._worker_busy = False
        self._pending_status: dict | None = None
        self._pending_generation = 0
        self._generation = 0
        self._applied_bias = 0

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
            obstacle_distance_cm=getattr(config, "obstacle_distance_cm", 20.0),
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
        if self.load_error:
            return

        actor_path = os.path.join(self.model_dir, "actor.ts")
        if not os.path.isfile(actor_path):
            self.load_error = "actor_ts_missing"
            return
        if self.observation_spec != "manifest":
            self.load_error = "manifest_observation_fields_missing"
            return
        has_mean_std = (
            len(self._mean) == len(self.observation_fields)
            and len(self._std) == len(self.observation_fields)
        )
        if (
            len(self._normalization_ranges) != len(self.observation_fields)
            and not has_mean_std
        ):
            self.load_error = "normalization_contract_missing"
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

        input_dimension = manifest.get("input_dimension")
        if input_dimension is not None:
            try:
                dimension = int(input_dimension)
            except (TypeError, ValueError):
                self.load_error = "manifest_input_dimension_invalid"
                return
            if dimension != len(self.observation_fields):
                self.load_error = "manifest_input_dimension_mismatch"
                return
        unsupported = [
            name for name in self.observation_fields if name not in _SUPPORTED_FIELDS
        ]
        if unsupported:
            self.load_error = f"unsupported_observation_field:{unsupported[0]}"
            return

        feature_clip = manifest.get("feature_clip")
        if isinstance(feature_clip, dict):
            self._feature_clip = self._range_mapping(feature_clip)

        scale = manifest.get("action_scale")
        if scale is not None:
            scale_value = _coerce_float(scale)
            if scale_value > 0:
                self.action_scale = scale_value

    def _load_normalization(self) -> None:
        payload = self._read_json("normalization.json")
        if not isinstance(payload, dict):
            return
        nested = payload.get("normalization")
        if isinstance(nested, dict):
            self._normalization_ranges = self._range_mapping(nested)
            if len(self._normalization_ranges) != len(self.observation_fields):
                self.load_error = "normalization_contract_incomplete"
                return
        self._mean = self._normalization_vector(payload.get("mean"), 0.0)
        self._std = self._normalization_vector(payload.get("std"), 1.0)

    def _range_mapping(self, raw: dict) -> dict[str, tuple[float, float]]:
        ranges: dict[str, tuple[float, float]] = {}
        for name in self.observation_fields:
            item = raw.get(name)
            if isinstance(item, dict):
                low = _finite_float(item.get("min"))
                high = _finite_float(item.get("max"))
            elif isinstance(item, (list, tuple)) and len(item) == 2:
                low = _finite_float(item[0])
                high = _finite_float(item[1])
            else:
                continue
            if low is not None and high is not None and high > low:
                ranges[name] = (low, high)
        return ranges

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
        explicit_heading_error = _finite_float(imu.get("fused_heading_error"))
        if explicit_heading_error is None:
            # Linear segments reset their relative odometry heading; zero is
            # therefore the straight-ahead target unless a controller supplies
            # an explicit target heading.
            target_heading = _coerce_float(
                status.get("target_heading_deg", imu.get("target_heading_deg", 0.0))
            )
            fused_heading_error = _wrap_degrees(target_heading - fused)
        else:
            fused_heading_error = _wrap_degrees(explicit_heading_error)

        left = _coerce_float(ultrasonic.get("left"))
        right = _coerce_float(ultrasonic.get("right"))
        left_cm = _coerce_float(encoders.get("left_cm"))
        right_cm = _coerce_float(encoders.get("right_cm"))
        front = _coerce_float(ultrasonic.get("center"))

        return {
            "motion_direction": (
                1.0 if action == "FORWARD" else -1.0 if action == "BACKWARD" else 0.0
            ),
            "segment_progress": max(0.0, min(1.0, progress_ratio)),
            "fused_heading_error": fused_heading_error,
            "left_right_encoder_error": left_cm - right_cm,
            "front_ultrasonic_distance": front,
            # Legacy bundles used the opposite sign under this old field name.
            "heading_error_deg": fused - encoder_heading,
            "fused_heading_deg": fused,
            "encoder_heading_deg": encoder_heading,
            "progress_ratio": max(0.0, min(1.0, progress_ratio)),
            "remaining_ticks": max(0.0, target_ticks - completed),
            "front_distance_cm": front,
            "left_distance_cm": left,
            "right_distance_cm": right,
            "lateral_offset_cm": left - right,
            "speed_correction": _coerce_float(imu.get("speed_correction")),
            "is_forward": 1.0 if action == "FORWARD" else 0.0,
            "is_backward": 1.0 if action == "BACKWARD" else 0.0,
        }

    def _vectorize(self, features: dict[str, float]) -> list[float]:
        vector = []
        for name in self.observation_fields:
            value = _coerce_float(features.get(name, 0.0))
            clip_range = self._feature_clip.get(name)
            if clip_range is not None:
                value = max(clip_range[0], min(clip_range[1], value))
            normalization_range = self._normalization_ranges.get(name)
            if normalization_range is not None:
                low, high = normalization_range
                value = 2.0 * (value - low) / (high - low) - 1.0
                value = max(-1.0, min(1.0, value))
            vector.append(value)
        if self._normalization_ranges:
            return vector
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
        with self._state_lock:
            self._generation += 1
            generation = self._generation
        result = self._evaluate(status, expected_generation=generation)
        with self._state_lock:
            if generation == self._generation:
                self._last_result = result
        return result

    def _eligibility_reason(self, status: dict) -> str | None:
        action = status.get("current_action") or ""
        if action == "BACKWARD":
            return "backward_motion"
        if action != "FORWARD":
            return "not_linear_motion"
        if status.get("state") != "MOVING":
            return "not_moving"

        ultrasonic = (status.get("telemetry") or {}).get("ultrasonic") or {}
        if ultrasonic.get("status") != "OK":
            return "telemetry_unavailable"
        front_distance = _finite_float(ultrasonic.get("center"))
        if front_distance is None or front_distance < 0:
            return "telemetry_unavailable"
        if front_distance < self.obstacle_distance_cm:
            return "obstacle_too_close"
        telemetry = status.get("telemetry") or {}
        encoders = telemetry.get("encoders") or {}
        imu = telemetry.get("imu") or {}
        if (
            _finite_float(encoders.get("left_cm")) is None
            or _finite_float(encoders.get("right_cm")) is None
            or _finite_float(imu.get("heading_fused_deg")) is None
        ):
            return "telemetry_unavailable"
        return None

    def _evaluate(
        self, status: dict, *, expected_generation: int | None = None
    ) -> dict:
        if self.mode == "disabled" or self._infer is None:
            return self._inactive_result(self.load_error or "disabled")

        eligibility_reason = self._eligibility_reason(status)
        if eligibility_reason is not None:
            self._clear_bias()
            return self._inactive_result(eligibility_reason)

        features = self.build_features(status)
        vector = self._vectorize(features)

        started = self._clock()
        try:
            raw_output = self._infer(vector)
        except Exception as exc:
            logger.warning("RL inference failed: %s", exc)
            self._clear_bias()
            return self._inactive_result(f"inference_failed:{type(exc).__name__}")
        inference_ms = (self._clock() - started) * 1000.0

        if not math.isfinite(raw_output):
            self._clear_bias()
            return self._inactive_result("invalid_output", inference_ms=inference_ms)

        clamped_output = max(-1.0, min(1.0, raw_output * self.action_scale))
        suggested = int(round(clamped_output * self.max_bias))
        suggested = max(-self.max_bias, min(self.max_bias, suggested))

        if inference_ms > self.deadline_ms:
            self._clear_bias()
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

        if expected_generation is not None:
            with self._state_lock:
                if expected_generation != self._generation:
                    return self._result(
                        model_output=raw_output,
                        suggested_bias=suggested,
                        applied_bias=0,
                        reason="stale_status",
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
            if setter(bias):
                self._applied_bias = bias
                return bias
            return 0
        except Exception as exc:
            logger.warning("Failed to send steer bias: %s", exc)
            return 0

    def _clear_bias(self) -> None:
        if self.mode != "active" or self._applied_bias == 0:
            return
        setter = getattr(self.serial, "set_steer_bias", None)
        if not callable(setter):
            self._applied_bias = 0
            return
        try:
            if setter(0):
                self._applied_bias = 0
        except Exception as exc:
            logger.warning("Failed to clear steer bias: %s", exc)

    # ---------------------------------------------------------- async worker

    def start(self) -> bool:
        """Start one latest-only inference worker; safe to call repeatedly."""
        if self.mode == "disabled" or self._infer is None:
            return False
        with self._state_lock:
            if self._worker is not None and self._worker.is_alive():
                return True
            self._worker_stop.clear()
            self._worker = threading.Thread(
                target=self._worker_loop,
                name="rl-residual-inference",
                daemon=True,
            )
            self._worker.start()
        return True

    def submit_status(self, status: dict) -> dict:
        """Queue the newest eligible status without blocking the control loop."""
        if self.mode == "disabled" or self._infer is None:
            result = self._inactive_result(self.load_error or "disabled")
            with self._state_lock:
                self._last_result = result
            return result

        reason = self._eligibility_reason(status)
        with self._state_lock:
            self._generation += 1
            generation = self._generation
            if reason is None:
                self._pending_status = deepcopy(status)
                self._pending_generation = generation
            else:
                self._pending_status = None
        if reason is not None:
            self._clear_bias()
            result = self._inactive_result(reason)
            with self._state_lock:
                self._last_result = result
            return result

        self.start()
        self._worker_event.set()
        with self._state_lock:
            return dict(self._last_result)

    def _worker_loop(self) -> None:
        while not self._worker_stop.is_set():
            self._worker_event.wait(0.1)
            if self._worker_stop.is_set():
                break
            with self._state_lock:
                status = self._pending_status
                generation = self._pending_generation
                self._pending_status = None
                self._worker_busy = status is not None
                self._worker_event.clear()
            if status is None:
                continue
            result = self._evaluate(status, expected_generation=generation)
            with self._state_lock:
                if generation == self._generation:
                    self._last_result = result
                self._worker_busy = False
                has_newer = self._pending_status is not None
            if has_newer:
                self._worker_event.set()

    def wait_until_idle(self, timeout: float = 1.0) -> bool:
        """Wait for test/shutdown observability; the application never calls this."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._state_lock:
                if self._pending_status is None and not self._worker_busy:
                    return True
            time.sleep(0.005)
        return False

    def stop(self) -> None:
        """Invalidate queued work and clear any active motor correction."""
        with self._state_lock:
            self._generation += 1
            self._pending_status = None
        self._clear_bias()
        self._worker_stop.set()
        self._worker_event.set()
        worker = self._worker
        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout=max(0.1, self.deadline_ms / 1000.0 * 2.0))

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
        with self._state_lock:
            status = dict(self._last_result)
            worker_running = self._worker is not None and self._worker.is_alive()
        status.update(
            requested_mode=self.requested_mode,
            backend=self._backend,
            model_dir=self.model_dir,
            model_loaded=self._infer is not None,
            load_error=self.load_error,
            max_bias=self.max_bias,
            deadline_ms=self.deadline_ms,
            obstacle_distance_cm=self.obstacle_distance_cm,
            observation_spec=self.observation_spec,
            observation_fields=list(self.observation_fields),
            normalization=(
                "min_max"
                if self._normalization_ranges
                else "mean_std"
                if self._mean or self._std
                else "none"
            ),
            worker_running=worker_running,
        )
        return status
