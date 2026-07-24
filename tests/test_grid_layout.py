import pytest

from pi.grid_layout import (
    BOX_IDS,
    EncoderCalibration,
    GridGeometry,
    build_grid_route,
    normalize_box_id,
)


def test_all_six_box_ids_are_available():
    assert BOX_IDS == ("1A", "1B", "2A", "2B", "3A", "3B")
    assert normalize_box_id(" 3b ") == "3B"
    with pytest.raises(ValueError, match="box_id"):
        normalize_box_id("4A")


def test_grid_route_is_generated_from_late_bound_dimensions():
    geometry = GridGeometry(80, 75, 35)
    calibration = EncoderCalibration(10, 420, 840)
    route = build_grid_route("3B", geometry, calibration)

    assert route["box_id"] == "3B"
    assert route["outbound"] == [
        {"action": "FORWARD", "target_ticks": 2300, "target_seconds": 0.0, "label": "Dock to row 3"},
        {"action": "TURN_RIGHT", "target_ticks": 420, "target_seconds": 0.0, "label": "Face box 3B"},
        {"action": "FORWARD", "target_ticks": 350, "target_seconds": 0.0, "label": "Approach box 3B"},
    ]
    assert [step["action"] for step in route["return"]] == [
        "BACKWARD", "TURN_LEFT", "BACKWARD"
    ]


def test_a_and_b_use_mirrored_turns():
    geometry = GridGeometry(80, 75, 35)
    calibration = EncoderCalibration(10, 420, 840)
    a_route = build_grid_route("1A", geometry, calibration)
    b_route = build_grid_route("1B", geometry, calibration)
    assert a_route["outbound"][1]["action"] == "TURN_LEFT"
    assert a_route["return"][1]["action"] == "TURN_RIGHT"
    assert b_route["outbound"][1]["action"] == "TURN_RIGHT"
    assert b_route["return"][1]["action"] == "TURN_LEFT"


def test_unmeasured_layout_refuses_to_guess_a_route():
    with pytest.raises(ValueError, match="not calibrated"):
        build_grid_route("1A", GridGeometry(), EncoderCalibration())


def test_invalid_box_is_rejected():
    with pytest.raises(ValueError, match="box_id"):
        normalize_box_id("5C")


def test_four_ticks_per_revolution_can_derive_linear_calibration():
    calibration = EncoderCalibration(
        turn_90_ticks=2,
        turn_180_ticks=4,
        ticks_per_revolution=4,
        wheel_diameter_cm=6.5,
    )
    assert calibration.missing_fields == []
    assert calibration.distance_ticks(100) == 20


def test_imu_turn_source_does_not_require_turn_tick_calibration():
    route = build_grid_route(
        "2A",
        GridGeometry(80, 75, 35),
        EncoderCalibration(
            ticks_per_revolution=4,
            wheel_diameter_cm=6.5,
        ),
        turn_source="imu",
    )
    assert route["outbound"][1]["target_ticks"] == 0


def test_imu_turns_do_not_silently_switch_straight_segments_to_timed_mode():
    route = build_grid_route(
        "1A",
        GridGeometry(80, 75, 35, forward_speed_cms=20),
        EncoderCalibration(
            ticks_per_revolution=4,
            wheel_diameter_cm=6.5,
        ),
        turn_source="imu",
    )

    assert route["outbound"][0]["target_ticks"] > 0
    assert route["outbound"][0]["target_seconds"] == 0.0
    assert route["return"][0]["target_ticks"] > 0
    assert route["return"][0]["action"] == "BACKWARD"


def test_timed_linear_mode_requires_explicit_selection():
    route = build_grid_route(
        "1B",
        GridGeometry(80, 75, 35, forward_speed_cms=20),
        EncoderCalibration(),
        turn_source="imu",
        linear_source="timed",
    )

    assert route["outbound"][0]["target_ticks"] == 0
    assert route["outbound"][0]["target_seconds"] == 4.0
    assert route["return"][-1]["action"] == "BACKWARD"
    assert route["return"][-1]["target_seconds"] == 4.0


def test_confirmed_encoder_and_wheel_defaults(monkeypatch):
    monkeypatch.delenv("LIBRARY_ROBOT_ENCODER_TICKS_PER_REV", raising=False)
    monkeypatch.delenv("LIBRARY_ROBOT_WHEEL_DIAMETER_CM", raising=False)
    calibration = EncoderCalibration.from_env()
    assert calibration.ticks_per_revolution == 4
    assert calibration.wheel_diameter_cm == 6.5
