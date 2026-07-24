import pytest
from tools.calibrate_fused_navigation import (
    calculate_ticks_per_cm,
    format_env_configuration,
    main,
)


def test_calculate_ticks_per_cm_normal():
    res = calculate_ticks_per_cm(100.0, 1000, 1000)
    assert res["left_ticks_per_cm"] == 10.0
    assert res["right_ticks_per_cm"] == 10.0
    assert res["avg_ticks_per_cm"] == 10.0
    assert res["asymmetry_percent"] == 0.0
    assert res["asymmetry_warning"] is False


def test_calculate_ticks_per_cm_asymmetry_warning():
    res = calculate_ticks_per_cm(100.0, 1000, 1100)
    assert res["asymmetry_percent"] > 5.0
    assert res["asymmetry_warning"] is True


def test_calculate_ticks_per_cm_invalid_input():
    with pytest.raises(ValueError, match="distance_cm"):
        calculate_ticks_per_cm(0, 100, 100)
    with pytest.raises(ValueError, match="Encoder ticks"):
        calculate_ticks_per_cm(100, -10, 100)


def test_format_env_configuration():
    res = calculate_ticks_per_cm(100.0, 1000, 1000)
    output = format_env_configuration(res)
    assert "LIBRARY_ROBOT_ENCODER_TICKS_PER_CM=10.0" in output
    assert "LIBRARY_ROBOT_LEFT_TICKS_PER_CM=10.0" in output
    assert "LIBRARY_ROBOT_RIGHT_TICKS_PER_CM=10.0" in output


def test_main_cli_help(capsys):
    assert main([]) == 0
    captured = capsys.readouterr()
    assert "Library Robot Fused Navigation Calibration Guide" in captured.out


def test_main_cli_args(capsys):
    assert main(["--distance", "100", "--left-ticks", "1000", "--right-ticks", "1000"]) == 0
    captured = capsys.readouterr()
    assert "Average Ticks/cm:   10.0" in captured.out
