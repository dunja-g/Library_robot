import pytest

from pi.navigation_config import NavigationConfig


def test_defaults_are_valid_for_current_camera_and_controller():
    config = NavigationConfig()
    assert (config.camera_width, config.camera_height) == (640, 480)
    assert config.target_confirmation_frames == 2
    assert config.target_loss_tolerance_frames == 3


def test_from_env_overrides_tuning_values(monkeypatch):
    monkeypatch.setenv("LIBRARY_ROBOT_CAMERA_FPS", "15")
    monkeypatch.setenv("LIBRARY_ROBOT_STOP_DISTANCE_CM", "28.5")
    monkeypatch.setenv("LIBRARY_ROBOT_MIN_MARKER_AREA_PX", "450")

    config = NavigationConfig.from_env()

    assert config.camera_fps == 15
    assert config.stop_distance_cm == 28.5
    assert config.min_marker_area_px == 450.0


def test_from_env_rejects_invalid_number(monkeypatch):
    monkeypatch.setenv("LIBRARY_ROBOT_CAMERA_WIDTH", "wide")
    with pytest.raises(ValueError, match="LIBRARY_ROBOT_CAMERA_WIDTH"):
        NavigationConfig.from_env()


def test_config_rejects_unsafe_values():
    with pytest.raises(ValueError):
        NavigationConfig(control_hz=0)
    with pytest.raises(ValueError):
        NavigationConfig(target_loss_tolerance_frames=-1)
