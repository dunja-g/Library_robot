"""Tests for RLResidualAdapter and its integration into Library_robot."""

import pytest
from pi.rl_residual_adapter import RLResidualAdapter


def test_rl_adapter_defaults_to_disabled():
    adapter = RLResidualAdapter(mode="disabled")
    assert adapter.mode == "disabled"

    res = adapter.step(
        is_forward=True,
        completed_distance=10.0,
        target_distance=50.0,
        target_heading_deg=0.0,
        fused_heading_deg=2.0,
        left_distance_cm=10.0,
        right_distance_cm=10.0,
        front_ultrasonic_cm=150.0,
        current_action="FORWARD",
    )
    assert res.valid is False
    assert res.apply_to_motor is False
    assert res.residual_pwm == 0
    assert res.reason == "disabled"


def test_rl_adapter_status_dict():
    adapter = RLResidualAdapter(mode="disabled")
    status = adapter.get_status()
    assert status["mode"] == "disabled"
    assert "model_loaded" in status
    assert "max_residual_pwm" in status
