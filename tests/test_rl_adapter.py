import json

import pytest

from pi.rl_residual_adapter import RLResidualAdapter


def moving_status(**overrides):
    status = {
        "state": "MOVING",
        "current_action": "FORWARD",
        "target_ticks": 100.0,
        "telemetry": {
            "encoders": {"left": 40, "right": 40},
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
