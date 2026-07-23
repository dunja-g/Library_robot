import pytest

from pi.grid_layout import (
    BOX_IDS,
    EncoderCalibration,
    GridGeometry,
    build_grid_route,
    normalize_box_id,
)


def test_all_eight_box_ids_are_available():
    assert BOX_IDS == ("1A", "1B", "2A", "2B", "3A", "3B", "4A", "4B")
    assert normalize_box_id(" 3b ") == "3B"


def test_grid_route_is_generated_from_late_bound_dimensions():
    geometry = GridGeometry(80, 75, 35)
    calibration = EncoderCalibration(10, 420, 840)
    route = build_grid_route("3B", geometry, calibration)

    assert route["box_id"] == "3B"
    assert route["outbound"] == [
        {"action": "FORWARD", "target_ticks": 2300, "label": "Dock to row 3"},
        {"action": "TURN_RIGHT", "target_ticks": 420, "label": "Face box 3B"},
        {"action": "FORWARD", "target_ticks": 350, "label": "Approach box 3B"},
    ]
    assert [step["action"] for step in route["return"]] == [
        "UTURN", "FORWARD", "TURN_LEFT", "FORWARD"
    ]


def test_a_and_b_use_mirrored_turns():
    geometry = GridGeometry(80, 75, 35)
    calibration = EncoderCalibration(10, 420, 840)
    a_route = build_grid_route("1A", geometry, calibration)
    b_route = build_grid_route("1B", geometry, calibration)
    assert a_route["outbound"][1]["action"] == "TURN_LEFT"
    assert a_route["return"][2]["action"] == "TURN_RIGHT"
    assert b_route["outbound"][1]["action"] == "TURN_RIGHT"
    assert b_route["return"][2]["action"] == "TURN_LEFT"


def test_unmeasured_layout_refuses_to_guess_a_route():
    with pytest.raises(ValueError, match="not calibrated"):
        build_grid_route("1A", GridGeometry(), EncoderCalibration())


def test_invalid_box_is_rejected():
    with pytest.raises(ValueError, match="box_id"):
        normalize_box_id("5C")
