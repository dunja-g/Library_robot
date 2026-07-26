import pytest

from pi.navigation_config import NavigationConfig


def test_defaults_are_valid_for_current_camera_and_controller():
    config = NavigationConfig()
    assert (config.camera_width, config.camera_height) == (640, 480)
    assert config.target_confirmation_frames == 2
    assert config.target_loss_tolerance_frames == 3
    assert config.invert_turn_direction is False


def test_from_env_overrides_tuning_values(monkeypatch):
    monkeypatch.setenv("LIBRARY_ROBOT_CAMERA_FPS", "15")
    monkeypatch.setenv("LIBRARY_ROBOT_STOP_DISTANCE_CM", "28.5")
    monkeypatch.setenv("LIBRARY_ROBOT_MIN_MARKER_AREA_PX", "450")
    monkeypatch.setenv("LIBRARY_ROBOT_AUTO_RETURN", "off")
    monkeypatch.setenv("LIBRARY_ROBOT_TURN_90_SECONDS", "1.1")
    monkeypatch.setenv("LIBRARY_ROBOT_ARUCO_STEERING_KP", "0.2")
    monkeypatch.setenv("LIBRARY_ROBOT_ARUCO_CANDIDATE_MAX_AREA_PX", "50000")
    monkeypatch.setenv("LIBRARY_ROBOT_ARUCO_CANDIDATE_MAX_JUMP_PX", "55")
    monkeypatch.setenv(
        "LIBRARY_ROBOT_LAST_ROW_RETURN_ALIGN_TOLERANCE_PX", "85"
    )
    monkeypatch.setenv("LIBRARY_ROBOT_RL_INVERT_BIAS", "true")

    config = NavigationConfig.from_env()

    assert config.camera_fps == 15
    assert config.stop_distance_cm == 28.5
    assert config.min_marker_area_px == 450.0
    assert config.auto_return is False
    assert config.turn_90_seconds == 1.1
    assert config.aruco_steering_kp == 0.2
    assert config.aruco_candidate_max_area_px == 50000
    assert config.aruco_candidate_max_jump_px == 55
    assert config.last_row_return_align_tolerance_px == 85
    assert config.rl_invert_bias is True


def test_from_env_rejects_invalid_number(monkeypatch):
    monkeypatch.setenv("LIBRARY_ROBOT_CAMERA_WIDTH", "wide")
    with pytest.raises(ValueError, match="LIBRARY_ROBOT_CAMERA_WIDTH"):
        NavigationConfig.from_env()


def test_from_env_rejects_invalid_boolean(monkeypatch):
    monkeypatch.setenv("LIBRARY_ROBOT_AUTO_RETURN", "sometimes")
    with pytest.raises(ValueError, match="LIBRARY_ROBOT_AUTO_RETURN"):
        NavigationConfig.from_env()


def test_config_rejects_unsafe_values():
    with pytest.raises(ValueError):
        NavigationConfig(control_hz=0)
    with pytest.raises(ValueError):
        NavigationConfig(target_loss_tolerance_frames=-1)
    with pytest.raises(ValueError):
        NavigationConfig(aruco_candidate_confirmation_frames=1)
