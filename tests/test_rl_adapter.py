import json
import os
import threading
import time

import pytest

from pi.rl_residual_adapter import RLResidualAdapter


def moving_status(**overrides):
    status = {
        "state": "MOVING",
        "current_action": "FORWARD",
        "target_ticks": 100.0,
        "telemetry": {
            "encoders": {
                "left": 40,
                "right": 40,
                "left_cm": 42.0,
                "right_cm": 40.0,
            },
            "imu": {
                "heading_fused_deg": 6.0,
                "heading_encoder_deg": 2.0,
                "speed_correction": 3,
            },
            "ultrasonic": {
                "status": "OK",
                "left": 22.0,
                "center": 90.0,
                "right": 18.0,
            },
        },
    }
    status.update(overrides)
    return status


class RecordingSerial:
    def __init__(self, ok=True):
        self.ok = ok
        self.biases = []

    def set_steer_bias(self, bias):
        self.biases.append(bias)
        return self.ok


def test_disabled_adapter_never_infers():
    calls = []
    adapter = RLResidualAdapter(
        mode="disabled",
        infer=lambda vector: calls.append(vector) or 1.0,
    )

    result = adapter.step_from_status(moving_status())

    assert calls == []
    assert result["enabled"] is False
    assert result["applied_bias"] == 0
    assert result["reason"] == "disabled"


def test_shadow_mode_reports_suggestion_without_touching_motors():
    serial = RecordingSerial()
    adapter = RLResidualAdapter(
        mode="shadow",
        max_bias=5,
        serial_bridge=serial,
        infer=lambda vector: 0.8,
    )

    result = adapter.step_from_status(moving_status())

    assert result["mode"] == "shadow"
    assert result["suggested_bias"] == 4
    assert result["applied_bias"] == 0
    assert result["reason"] == "shadow_mode"
    assert serial.biases == []


def test_active_mode_sends_clamped_bias():
    serial = RecordingSerial()
    adapter = RLResidualAdapter(
        mode="active",
        max_bias=5,
        serial_bridge=serial,
        infer=lambda vector: 9.0,
    )

    result = adapter.step_from_status(moving_status())

    assert result["suggested_bias"] == 5
    assert result["applied_bias"] == 5
    assert result["reason"] == "applied"
    assert serial.biases == [5]


def test_active_mode_can_invert_bias_for_chassis_calibration():
    serial = RecordingSerial()
    adapter = RLResidualAdapter(
        mode="active",
        max_bias=5,
        invert_bias=True,
        serial_bridge=serial,
        infer=lambda vector: 0.8,
    )

    result = adapter.step_from_status(moving_status())

    assert result["suggested_bias"] == -4
    assert result["applied_bias"] == -4
    assert serial.biases == [-4]
    assert adapter.get_status()["invert_bias"] is True


def test_active_mode_reports_failure_when_serial_rejects():
    serial = RecordingSerial(ok=False)
    adapter = RLResidualAdapter(
        mode="active",
        serial_bridge=serial,
        infer=lambda vector: -1.0,
    )

    result = adapter.step_from_status(moving_status())

    assert result["applied_bias"] == 0
    assert result["reason"] == "apply_failed"


@pytest.mark.parametrize(
    "status, expected_reason",
    [
        (moving_status(current_action="TURN_LEFT"), "not_linear_motion"),
        (moving_status(state="ARRIVED"), "not_moving"),
    ],
)
def test_adapter_skips_non_linear_phases(status, expected_reason):
    serial = RecordingSerial()
    adapter = RLResidualAdapter(
        mode="active",
        serial_bridge=serial,
        infer=lambda vector: 1.0,
    )

    assert adapter.step_from_status(status)["reason"] == expected_reason
    assert serial.biases == []


def test_adapter_skips_when_ultrasonic_telemetry_is_stale():
    status = moving_status()
    status["telemetry"]["ultrasonic"]["status"] = "WAITING"
    adapter = RLResidualAdapter(mode="shadow", infer=lambda vector: 1.0)

    assert adapter.step_from_status(status)["reason"] == "telemetry_unavailable"


def test_backward_immediately_clears_an_active_bias():
    serial = RecordingSerial()
    adapter = RLResidualAdapter(
        mode="active",
        serial_bridge=serial,
        infer=lambda vector: 1.0,
    )
    adapter.step_from_status(moving_status())

    result = adapter.step_from_status(moving_status(current_action="BACKWARD"))

    assert result["reason"] == "backward_motion"
    assert serial.biases == [5, 0]


def test_obstacle_distance_blocks_inference_and_clears_bias():
    calls = []
    serial = RecordingSerial()
    adapter = RLResidualAdapter(
        mode="active",
        obstacle_distance_cm=20.0,
        serial_bridge=serial,
        infer=lambda vector: calls.append(vector) or 1.0,
    )
    adapter.step_from_status(moving_status())
    blocked = moving_status()
    blocked["telemetry"]["ultrasonic"]["center"] = 10.0

    result = adapter.step_from_status(blocked)

    assert result["reason"] == "obstacle_too_close"
    assert len(calls) == 1
    assert serial.biases == [5, 0]


def test_slow_inference_is_reported_but_not_applied():
    class StepClock:
        def __init__(self):
            self.now = 0.0

        def __call__(self):
            value = self.now
            self.now += 0.2
            return value

    serial = RecordingSerial()
    adapter = RLResidualAdapter(
        mode="active",
        deadline_ms=50.0,
        serial_bridge=serial,
        infer=lambda vector: 1.0,
        clock=StepClock(),
    )

    result = adapter.step_from_status(moving_status())

    assert result["reason"] == "deadline_exceeded"
    assert result["applied_bias"] == 0
    assert serial.biases == []


def test_non_finite_output_is_rejected():
    adapter = RLResidualAdapter(mode="shadow", infer=lambda vector: float("nan"))

    result = adapter.step_from_status(moving_status())

    assert result["reason"] == "invalid_output"
    assert result["suggested_bias"] == 0


def test_inference_exception_does_not_propagate():
    def explode(vector):
        raise RuntimeError("boom")

    adapter = RLResidualAdapter(mode="shadow", infer=explode)

    result = adapter.step_from_status(moving_status())

    assert result["reason"] == "inference_failed:RuntimeError"


def test_features_are_derived_from_controller_telemetry():
    features = RLResidualAdapter.build_features(moving_status())

    assert features["heading_error_deg"] == pytest.approx(4.0)
    assert features["motion_direction"] == 1.0
    assert features["segment_progress"] == pytest.approx(0.4)
    assert features["fused_heading_error"] == pytest.approx(-6.0)
    assert features["left_right_encoder_error"] == pytest.approx(2.0)
    assert features["front_ultrasonic_distance"] == pytest.approx(90.0)
    assert features["progress_ratio"] == pytest.approx(0.4)
    assert features["front_distance_cm"] == pytest.approx(90.0)
    assert features["lateral_offset_cm"] == pytest.approx(4.0)
    assert features["is_forward"] == 1.0


def test_manifest_defines_observation_order_and_normalization(tmp_path):
    (tmp_path / "manifest.json").write_text(
        json.dumps(
            {
                "observation_fields": ["front_distance_cm", "heading_error_deg"],
                "action_scale": 1.0,
            }
        ),
        encoding="utf-8",
    )
    (tmp_path / "normalization.json").write_text(
        json.dumps({"mean": [90.0, 0.0], "std": [10.0, 2.0]}),
        encoding="utf-8",
    )

    seen = []

    adapter = RLResidualAdapter(mode="shadow", model_dir=str(tmp_path))
    adapter._load_manifest()
    adapter._load_normalization()
    adapter._infer = lambda vector: seen.append(list(vector)) or 0.0
    adapter.mode = "shadow"

    adapter.step_from_status(moving_status())

    assert adapter.observation_spec == "manifest"
    assert adapter.observation_fields == ("front_distance_cm", "heading_error_deg")
    assert seen == [[0.0, 2.0]]


def test_feature_names_and_nested_min_max_match_library_bundle(tmp_path):
    feature_names = [
        "motion_direction",
        "segment_progress",
        "fused_heading_error",
        "left_right_encoder_error",
        "front_ultrasonic_distance",
    ]
    (tmp_path / "manifest.json").write_text(
        json.dumps(
            {
                "feature_names": feature_names,
                "input_dimension": 5,
                "feature_clip": {
                    "motion_direction": [-1, 1],
                    "segment_progress": [0, 1],
                    "fused_heading_error": [-180, 180],
                    "left_right_encoder_error": [-50, 50],
                    "front_ultrasonic_distance": [0, 400],
                },
            }
        ),
        encoding="utf-8",
    )
    (tmp_path / "normalization.json").write_text(
        json.dumps(
            {
                "normalization": {
                    "motion_direction": {"min": -1, "max": 1},
                    "segment_progress": {"min": 0, "max": 1},
                    "fused_heading_error": {"min": -45, "max": 45},
                    "left_right_encoder_error": {"min": -10, "max": 10},
                    "front_ultrasonic_distance": {"min": 0, "max": 400},
                }
            }
        ),
        encoding="utf-8",
    )
    adapter = RLResidualAdapter(mode="shadow", model_dir=str(tmp_path))
    adapter._load_manifest()
    adapter._load_normalization()

    vector = adapter._vectorize(adapter.build_features(moving_status()))

    assert adapter.observation_spec == "manifest"
    assert adapter.observation_fields == tuple(feature_names)
    assert vector == pytest.approx([1.0, -0.2, -6 / 45, 0.2, -0.55])
    assert adapter.get_status()["normalization"] == "min_max"


def test_latest_only_worker_does_not_block_and_discards_intermediate_status():
    started = threading.Event()
    release = threading.Event()
    seen = []

    def slow_infer(vector):
        seen.append(list(vector))
        started.set()
        release.wait(1.0)
        return 0.0

    adapter = RLResidualAdapter(mode="shadow", infer=slow_infer)
    adapter.start()
    first = moving_status()
    first["target_ticks"] = 100
    first["telemetry"]["encoders"].update(left=10, right=10)
    started_at = time.monotonic()
    adapter.submit_status(first)
    elapsed = time.monotonic() - started_at
    assert elapsed < 0.05
    assert started.wait(1.0)

    middle = moving_status()
    middle["telemetry"]["encoders"].update(left=20, right=20)
    newest = moving_status()
    newest["telemetry"]["encoders"].update(left=80, right=80)
    adapter.submit_status(middle)
    adapter.submit_status(newest)
    release.set()

    assert adapter.wait_until_idle(1.0)
    adapter.stop()
    assert len(seen) == 2
    assert seen[0][1] == pytest.approx(0.1)
    assert seen[1][1] == pytest.approx(0.8)


def test_background_result_cannot_apply_after_backward_transition():
    started = threading.Event()
    release = threading.Event()
    serial = RecordingSerial()

    def slow_infer(vector):
        started.set()
        release.wait(1.0)
        return 1.0

    adapter = RLResidualAdapter(
        mode="active", serial_bridge=serial, infer=slow_infer
    )
    adapter.start()
    adapter.submit_status(moving_status())
    assert started.wait(1.0)

    result = adapter.submit_status(moving_status(current_action="BACKWARD"))
    release.set()

    assert result["reason"] == "backward_motion"
    assert adapter.wait_until_idle(1.0)
    adapter.stop()
    assert serial.biases == []


def test_real_library_sac_bundle_contract_when_available():
    model_dir = os.getenv("LIBRARY_ROBOT_RL_TEST_MODEL_DIR")
    if not model_dir:
        pytest.skip("set LIBRARY_ROBOT_RL_TEST_MODEL_DIR to library_sac_best")

    adapter = RLResidualAdapter(
        mode="shadow", model_dir=model_dir, deadline_ms=5_000
    )

    assert adapter.load_error is None
    assert adapter.observation_fields == (
        "motion_direction",
        "segment_progress",
        "fused_heading_error",
        "left_right_encoder_error",
        "front_ultrasonic_distance",
    )
    assert adapter.get_status()["normalization"] == "min_max"
    result = adapter.step_from_status(moving_status())
    assert result["reason"] == "shadow_mode"
    assert result["inference_ms"] <= adapter.deadline_ms


def test_missing_bundle_reports_load_error_and_stays_disabled(tmp_path):
    adapter = RLResidualAdapter(
        mode="shadow", model_dir=str(tmp_path / "nope")
    )

    assert adapter.mode == "disabled"
    assert adapter.load_error == "model_dir_missing"
    assert adapter.get_status()["requested_mode"] == "shadow"


def test_missing_actor_file_is_reported(tmp_path):
    adapter = RLResidualAdapter(mode="shadow", model_dir=str(tmp_path))

    assert adapter.load_error == "actor_ts_missing"
    assert adapter.get_status()["model_loaded"] is False


def test_status_exposes_observation_contract():
    adapter = RLResidualAdapter(mode="shadow", infer=lambda vector: 0.0)

    status = adapter.get_status()

    assert status["backend"] == "injected"
    assert status["observation_spec"] == "assumed_default"
    assert len(status["observation_fields"]) == 5


def test_invalid_mode_is_rejected():
    with pytest.raises(ValueError):
        RLResidualAdapter(mode="turbo")
