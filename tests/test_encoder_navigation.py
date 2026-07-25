import pytest
import numpy as np

from pi.encoder_navigation import GridController, GridState
from pi.grid_layout import EncoderCalibration, GridGeometry, build_grid_route


class FakeClock:
    def __init__(self): self.now = 0.0
    def __call__(self): return self.now


class FakeSerial:
    def __init__(self):
        self.commands = []
        self.encoders = {"left": 0, "right": 0}
        self.ultrasonic = {"left": 100, "center": 100, "right": 100}
        self.reset_ok = True
        self.motion_ok = True
        self.turn_status = "ACTIVE"
    def send_stop(self): self.commands.append("STOP"); return True
    def send_forward(self): self.commands.append("FORWARD"); return self.motion_ok
    def send_backward(self): self.commands.append("BACKWARD"); return self.motion_ok
    def send_rotate_left(self): self.commands.append("ROTATE_LEFT"); return True
    def send_rotate_right(self): self.commands.append("ROTATE_RIGHT"); return True
    def reset_encoders(self):
        self.commands.append("ENC_RESET")
        self.encoders = {"left": 0, "right": 0}
        return self.reset_ok
    def get_encoders(self): return self.encoders
    def get_odometry(self):
        if self.encoders is None:
            return None
        return {
            **self.encoders,
            "left_cm": float(self.encoders["left"]),
            "right_cm": float(self.encoders["right"]),
            "heading_encoder_deg": 1.0,
            "heading_imu_deg": 2.0,
            "heading_fused_deg": 1.95,
            "speed_correction": -4,
        }
    def get_ultrasonic(self): return self.ultrasonic
    def send_turn_left(self, _degrees=None): self.commands.append("TURN_LEFT"); return True
    def send_turn_right(self, _degrees=None): self.commands.append("TURN_RIGHT"); return True
    def send_turn_uturn(self, _degrees=None): self.commands.append("TURN_UTURN"); return True
    def get_turn_status(self): return self.turn_status


def make_controller():
    serial, clock = FakeSerial(), FakeClock()
    controller = GridController(
        serial, clock=clock, destination_dwell_seconds=1,
        encoder_stall_seconds=2,
        invert_turn_direction=False,
    )
    plan = build_grid_route(
        "1A", GridGeometry(10, 10, 5), EncoderCalibration(1, 4, 8),
        vision_source="encoder",
    )
    controller.request_grid_mission(plan)
    return controller, serial, clock


def complete_step(controller, serial):
    target = controller.get_status()["target_ticks"]
    serial.encoders = {"left": target, "right": target}
    controller.step()


def test_full_encoder_route_reaches_box_then_dock():
    controller, serial, clock = make_controller()
    assert controller.get_state() == GridState.MOVING.value

    for _ in range(3):
        complete_step(controller, serial)
    assert controller.get_state() == GridState.ARRIVED.value
    assert controller.get_status()["phase"] == "AT_DESTINATION"

    clock.now = 1
    controller.step()
    assert controller.get_status()["phase"] == "RETURNING"
    for _ in range(3):
        complete_step(controller, serial)
    assert controller.get_state() == GridState.DOCKED.value
    assert controller.get_status()["reason"] == "dock_reached"


def test_obstacle_stops_before_encoder_motion_continues():
    controller, serial, _clock = make_controller()
    serial.ultrasonic["center"] = 10
    controller.step()
    assert controller.get_state() == GridState.STOPPED.value
    assert controller.get_status()["reason"] == "center_obstacle"


def test_forward_ignores_side_obstacle_readings():
    controller, serial, _clock = make_controller()
    serial.ultrasonic["left"] = 5
    serial.ultrasonic["right"] = 5
    controller.step()
    assert controller.get_state() == GridState.MOVING.value


def test_invert_turn_direction_swaps_rotate_commands():
    serial, clock = FakeSerial(), FakeClock()
    controller = GridController(
        serial,
        clock=clock,
        invert_turn_direction=True,
    )
    plan = build_grid_route(
        "1A", GridGeometry(10, 10, 5), EncoderCalibration(1, 4, 8),
        vision_source="encoder",
    )
    controller.request_grid_mission(plan)
    controller.step_index = 1
    controller._start_current_step()
    controller.step()
    assert serial.commands[-1] == "ROTATE_RIGHT"


def test_reverse_escape_ignores_side_shelf_readings_during_return():
    controller, serial, clock = make_controller()
    for _ in range(3):
        complete_step(controller, serial)
    clock.now = 1
    controller.step()
    assert controller.get_status()["phase"] == "RETURNING"
    assert controller.get_status()["current_action"] == "BACKWARD"

    serial.ultrasonic["left"] = 10
    serial.ultrasonic["right"] = 8
    controller.step()

    assert controller.get_state() == GridState.MOVING.value


def test_reverse_ignores_front_shelf_but_requires_valid_sensor_data():
    controller, serial, clock = make_controller()
    for _ in range(3):
        complete_step(controller, serial)
    clock.now = 1
    controller.step()
    serial.ultrasonic["center"] = 5
    controller.step()
    assert controller.get_state() == GridState.MOVING.value

    serial.ultrasonic = None
    controller.step()
    assert controller.get_status()["reason"] == "ultrasonic_unavailable"


def test_missing_encoder_data_fails_safe():
    controller, serial, _clock = make_controller()
    serial.encoders = None
    controller.step()
    assert controller.get_status()["reason"] == "encoder_unavailable"


def test_serial_motion_command_failure_stops_mission():
    serial, clock = FakeSerial(), FakeClock()
    serial.motion_ok = False
    controller = GridController(serial, clock=clock)
    plan = build_grid_route(
        "1A", GridGeometry(10, 10, 5), EncoderCalibration(1, 4, 8),
        vision_source="encoder",
    )

    controller.request_grid_mission(plan)

    assert controller.get_state() == GridState.STOPPED.value
    assert controller.get_status()["reason"] == "serial_command_failed"


def test_encoder_stall_is_detected_without_sleeping():
    controller, serial, clock = make_controller()
    controller.step()
    clock.now = 2
    controller.step()
    assert controller.get_status()["reason"] == "encoder_stall"


def test_one_stalled_drivetrain_side_cannot_be_hidden_by_other_encoder():
    controller, serial, clock = make_controller()
    serial.encoders = {"left": 100, "right": 0}
    controller.step()
    clock.now = 2
    serial.encoders = {"left": 200, "right": 0}
    controller.step()
    assert controller.get_status()["reason"] == "encoder_stall"


def test_status_exposes_dashboard_sensor_telemetry():
    controller, serial, _clock = make_controller()
    serial.encoders = {"left": 2, "right": 3}
    controller.step()
    telemetry = controller.get_status()["telemetry"]
    assert telemetry["encoders"] == {
        "status": "OK",
        "left": 2,
        "right": 3,
        "left_cm": 2.0,
        "right_cm": 3.0,
        "distance_cm": 2.5,
    }
    assert telemetry["ultrasonic"] == {
        "status": "OK", "left": 100, "center": 100, "right": 100
    }
    assert telemetry["imu"]["heading_encoder_deg"] == 1.0
    assert telemetry["imu"]["heading_imu_deg"] == 2.0
    assert telemetry["imu"]["heading_fused_deg"] == 1.95
    assert telemetry["imu"]["speed_correction"] == -4
    assert 0 <= telemetry["segment_progress_percent"] <= 100
    status = controller.get_status()
    assert status["active_controller"] == "GridController"
    assert status["return_strategy"] == "direct_reverse"
    assert status["return_actions"] == ["BACKWARD", "TURN_RIGHT", "BACKWARD"]


def test_reset_cancels_grid_plan_and_stops():
    controller, serial, _clock = make_controller()
    controller.reset()
    assert controller.get_state() == GridState.IDLE.value
    assert controller.plan is None
    assert serial.commands[-1] == "STOP"


def test_confirmation_mission_waits_at_shelf_before_reverse_return():
    serial, clock = FakeSerial(), FakeClock()
    controller = GridController(serial, clock=clock, destination_dwell_seconds=0)
    plan = build_grid_route(
        "1A", GridGeometry(10, 10, 5), EncoderCalibration(1, 4, 8),
        vision_source="encoder",
    )
    plan["pickup_confirmation_required"] = True
    controller.request_grid_mission(plan)
    for _ in range(3):
        complete_step(controller, serial)

    clock.now = 100
    controller.step()
    assert controller.get_state() == GridState.ARRIVED.value
    assert controller.get_status()["pickup_confirmation_required"] is True

    controller.confirm_pickup()
    assert controller.get_status()["phase"] == "RETURNING"
    assert controller.get_status()["current_action"] == "BACKWARD"


def test_duplicate_grid_mission_is_rejected():
    controller, _serial, _clock = make_controller()
    duplicate = build_grid_route(
        "2A", GridGeometry(10, 10, 5), EncoderCalibration(1, 4, 8),
        vision_source="encoder",
    )
    with pytest.raises(RuntimeError, match="already active"):
        controller.request_grid_mission(duplicate)


def test_imu_turn_source_does_not_depend_on_four_tick_encoder_turns():
    serial, clock = FakeSerial(), FakeClock()
    controller = GridController(
        serial, clock=clock, destination_dwell_seconds=0, turn_source="imu",
        invert_turn_direction=False,
    )
    plan = build_grid_route(
        "1B", GridGeometry(10, 10, 5), EncoderCalibration(1, 4, 8),
        vision_source="encoder",
        turn_source="imu",
    )
    controller.request_grid_mission(plan)
    complete_step(controller, serial)
    assert controller.get_state() == GridState.TURNING.value
    assert serial.commands[-1] == "TURN_RIGHT"
    serial.turn_status = "DONE"
    controller.step()
    assert controller.get_state() == GridState.MOVING.value


def finish_align_pulse(controller, clock):
    clock.now += controller.aruco_align_pulse_seconds + 0.01
    controller.step()


def finish_align_settle(controller, clock):
    clock.now += controller.aruco_align_settle_seconds + 0.01
    controller.step()


def finish_align_fine_settle(controller, clock):
    clock.now += controller.aruco_align_fine_settle_seconds + 0.01
    controller.step()


class CenteredArucoDetector:
    def detect_target(self, frame, target_id):
        return {
            "id": target_id,
            "center_x": 320,
            "center_y": 240,
            "area": 100000,
        }


class MissingArucoDetector:
    def detect_target(self, frame, target_id):
        return None


def test_aruco_align_looks_before_search_rotation():
    serial, clock = FakeSerial(), FakeClock()
    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    controller = GridController(
        serial,
        clock=clock,
        frame_provider=lambda: frame,
        aruco_detector=MissingArucoDetector(),
        alignment_confirmation_frames=1,
        aruco_align_pulse_seconds=0.2,
        invert_turn_direction=False,
    )
    plan = build_grid_route(
        "1A",
        GridGeometry(10, 10, 5),
        EncoderCalibration(1, 4, 8),
        turn_source="imu",
    )
    controller.request_grid_mission(plan)

    controller.step()
    assert serial.commands[-1] == "STOP"
    assert controller._align_settle_until is not None
    assert "ROTATE" not in serial.commands

    finish_align_settle(controller, clock)
    assert serial.commands[-1] == "ROTATE_LEFT"
    assert controller._align_pulse_deadline is not None

    finish_align_pulse(controller, clock)
    assert serial.commands[-1] == "STOP"
    assert controller._align_settle_until is not None


class OffCenterArucoDetector:
    def detect_target(self, frame, target_id):
        return {
            "id": target_id,
            "center_x": 400,
            "center_y": 240,
            "area": 100000,
        }


def test_aruco_align_fine_pulses_when_marker_is_off_center():
    serial, clock = FakeSerial(), FakeClock()
    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    controller = GridController(
        serial,
        clock=clock,
        frame_provider=lambda: frame,
        aruco_detector=OffCenterArucoDetector(),
        alignment_confirmation_frames=1,
        aruco_align_pulse_seconds=0.2,
        aruco_align_fine_pulse_seconds=0.12,
        invert_turn_direction=False,
    )
    plan = build_grid_route(
        "1A",
        GridGeometry(10, 10, 5),
        EncoderCalibration(1, 4, 8),
        turn_source="imu",
    )
    controller.request_grid_mission(plan)

    finish_align_settle(controller, clock)
    controller.step()
    assert serial.commands[-1] == "ROTATE_RIGHT"
    assert controller._align_pulse_deadline is not None


def test_aruco_align_turns_toward_marker_despite_route_turn_inversion():
    serial, clock = FakeSerial(), FakeClock()
    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    controller = GridController(
        serial,
        clock=clock,
        frame_provider=lambda: frame,
        aruco_detector=OffCenterArucoDetector(),
        alignment_confirmation_frames=1,
        invert_turn_direction=True,
    )
    plan = build_grid_route(
        "1A",
        GridGeometry(10, 10, 5),
        EncoderCalibration(1, 4, 8),
        turn_source="imu",
    )
    controller.request_grid_mission(plan)

    finish_align_settle(controller, clock)
    controller.step()

    # Marker sits right of centre, so the robot must physically turn right
    # even though route turns are inverted for this chassis.
    assert serial.commands[-1] == "ROTATE_RIGHT"


class UndecodableCandidateDetector:
    """Never decodes an ID but always reports a quad right of centre."""

    def detect_target(self, frame, target_id):
        return None

    def detect(self, frame):
        return []

    def detect_candidates(self, frame):
        return [{"center_x": 520, "center_y": 240, "area": 12000}]


def test_aruco_align_steers_toward_an_undecodable_code():
    serial, clock = FakeSerial(), FakeClock()
    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    controller = GridController(
        serial,
        clock=clock,
        frame_provider=lambda: frame,
        aruco_detector=UndecodableCandidateDetector(),
        alignment_confirmation_frames=1,
        invert_turn_direction=False,
    )
    plan = build_grid_route(
        "1A",
        GridGeometry(10, 10, 5),
        EncoderCalibration(1, 4, 8),
        turn_source="imu",
    )
    controller.request_grid_mission(plan)

    finish_align_settle(controller, clock)

    assert serial.commands[-1] == "ROTATE_RIGHT"
    assert controller._align_pulse_deadline is not None
    assert controller._align_search_total == 0


def test_candidate_tracking_can_be_disabled():
    serial, clock = FakeSerial(), FakeClock()
    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    controller = GridController(
        serial,
        clock=clock,
        frame_provider=lambda: frame,
        aruco_detector=UndecodableCandidateDetector(),
        alignment_confirmation_frames=1,
        aruco_track_candidates=False,
        invert_turn_direction=False,
    )
    plan = build_grid_route(
        "1A",
        GridGeometry(10, 10, 5),
        EncoderCalibration(1, 4, 8),
        turn_source="imu",
    )
    controller.request_grid_mission(plan)

    finish_align_settle(controller, clock)

    assert controller._align_search_total == 1


def test_aruco_align_search_alternates_when_marker_stays_missing():
    serial, clock = FakeSerial(), FakeClock()
    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    controller = GridController(
        serial,
        clock=clock,
        frame_provider=lambda: frame,
        aruco_detector=MissingArucoDetector(),
        alignment_confirmation_frames=1,
        aruco_align_pulse_seconds=0.2,
        invert_turn_direction=False,
    )
    plan = build_grid_route(
        "1A",
        GridGeometry(10, 10, 5),
        EncoderCalibration(1, 4, 8),
        turn_source="imu",
    )
    controller.request_grid_mission(plan)

    finish_align_settle(controller, clock)
    assert serial.commands[-1] == "ROTATE_LEFT"

    finish_align_pulse(controller, clock)
    finish_align_settle(controller, clock)
    assert serial.commands[-1] == "ROTATE_RIGHT"


class FlickerOffCenterArucoDetector:
    def __init__(self):
        self.calls = 0

    def detect_target(self, frame, target_id):
        self.calls += 1
        if self.calls == 1:
            return {
                "id": target_id,
                "center_x": 500,
                "center_y": 240,
                "area": 100000,
            }
        if self.calls in {2, 3}:
            return None
        return {
            "id": target_id,
            "center_x": 500,
            "center_y": 240,
            "area": 100000,
        }


def test_aruco_align_reacquires_toward_last_offset_after_brief_loss():
    serial, clock = FakeSerial(), FakeClock()
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    detector = FlickerOffCenterArucoDetector()

    controller = GridController(
        serial,
        clock=clock,
        frame_provider=lambda: frame,
        aruco_detector=detector,
        alignment_confirmation_frames=1,
        aruco_align_pulse_seconds=0.2,
        aruco_align_fine_pulse_seconds=0.12,
        aruco_align_fine_settle_seconds=0.4,
        invert_turn_direction=False,
    )
    plan = build_grid_route(
        "1A",
        GridGeometry(10, 10, 5),
        EncoderCalibration(1, 4, 8),
        turn_source="imu",
    )
    controller.request_grid_mission(plan)

    finish_align_settle(controller, clock)
    controller.step()
    assert serial.commands[-1] == "ROTATE_RIGHT"

    clock.now += controller.aruco_align_fine_pulse_seconds + 0.01
    controller.step()
    finish_align_fine_settle(controller, clock)
    assert serial.commands[-1] == "ROTATE_RIGHT"


class VanishingOffCenterArucoDetector:
    def __init__(self):
        self.calls = 0

    def detect_target(self, frame, target_id):
        self.calls += 1
        if self.calls > 1:
            return None
        return {
            "id": target_id,
            "center_x": 500,
            "center_y": 240,
            "area": 100000,
        }


def test_aruco_align_stops_chasing_one_direction_after_reacquire_budget():
    serial, clock = FakeSerial(), FakeClock()
    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    controller = GridController(
        serial,
        clock=clock,
        frame_provider=lambda: frame,
        aruco_detector=VanishingOffCenterArucoDetector(),
        alignment_confirmation_frames=1,
        aruco_align_pulse_seconds=0.2,
        aruco_align_fine_pulse_seconds=0.12,
        aruco_align_fine_settle_seconds=0.4,
        aruco_align_max_reacquire_pulses=1,
        invert_turn_direction=False,
    )
    plan = build_grid_route(
        "1A",
        GridGeometry(10, 10, 5),
        EncoderCalibration(1, 4, 8),
        turn_source="imu",
    )
    controller.request_grid_mission(plan)

    finish_align_settle(controller, clock)
    controller.step()
    assert serial.commands[-1] == "ROTATE_RIGHT"

    rotations = []
    for _ in range(6):
        if controller._align_pulse_deadline is not None:
            clock.now = controller._align_pulse_deadline + 0.01
        elif controller._align_settle_until is not None:
            clock.now = controller._align_settle_until + 0.01
        controller.step()
        if serial.commands[-1].startswith("ROTATE"):
            rotations.append(serial.commands[-1])

    assert "ROTATE_LEFT" in rotations


def test_aruco_align_gives_up_instead_of_spinning_when_marker_never_appears():
    serial, clock = FakeSerial(), FakeClock()
    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    controller = GridController(
        serial,
        clock=clock,
        frame_provider=lambda: frame,
        aruco_detector=MissingArucoDetector(),
        alignment_confirmation_frames=1,
        aruco_align_pulse_seconds=0.2,
        aruco_align_max_search_pulses=3,
        invert_turn_direction=False,
    )
    plan = build_grid_route(
        "1A",
        GridGeometry(10, 10, 5),
        EncoderCalibration(1, 4, 8),
        turn_source="imu",
    )
    controller.request_grid_mission(plan)

    for _ in range(20):
        if controller._align_pulse_deadline is not None:
            clock.now = controller._align_pulse_deadline + 0.01
        elif controller._align_settle_until is not None:
            clock.now = controller._align_settle_until + 0.01
        controller.step()
        if controller.get_state() == GridState.STOPPED.value:
            break

    assert controller.get_state() == GridState.STOPPED.value
    assert controller.get_status()["reason"] == "aruco_marker_not_found"
    assert serial.commands.count("ROTATE_LEFT") + serial.commands.count(
        "ROTATE_RIGHT"
    ) == 3


def test_aruco_align_advances_after_stop_and_detection():
    serial, clock = FakeSerial(), FakeClock()
    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    controller = GridController(
        serial,
        clock=clock,
        frame_provider=lambda: frame,
        aruco_detector=CenteredArucoDetector(),
        alignment_confirmation_frames=1,
        aruco_align_pulse_seconds=0.2,
    )
    plan = build_grid_route(
        "1A",
        GridGeometry(10, 10, 5),
        EncoderCalibration(1, 4, 8),
        turn_source="imu",
    )
    controller.request_grid_mission(plan)

    finish_align_settle(controller, clock)
    controller.step()
    assert controller.get_status()["current_action"] == "FORWARD"


def test_aruco_approach_creeps_forward_after_target_area():
    serial, clock = FakeSerial(), FakeClock()
    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    controller = GridController(
        serial,
        clock=clock,
        frame_provider=lambda: frame,
        aruco_detector=CenteredArucoDetector(),
        aruco_target_area_px=8000.0,
        aruco_approach_extra_ticks=12.0,
        alignment_confirmation_frames=1,
    )
    plan = build_grid_route(
        "1A",
        GridGeometry(10, 10, 5),
        EncoderCalibration(1, 4, 8),
        turn_source="imu",
    )
    controller.request_grid_mission(plan)
    controller.step_index = 4
    controller._start_current_step()

    controller.step()
    assert controller._current_step()["aruco_creep_active"] is True
    assert "ENC_RESET" in serial.commands
    assert serial.commands[-1] == "FORWARD"

    serial.encoders = {"left": 12, "right": 12}
    controller.step()
    assert serial.commands[-1] == "STOP"
    assert controller.get_state() == GridState.ARRIVED.value


def test_encoder_forward_applies_soft_aruco_tracking():
    serial, clock = FakeSerial(), FakeClock()
    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    class TrackingSerial(FakeSerial):
        def set_trim(self, value):
            self.commands.append(f"TRIM={value}")
            return True

    serial = TrackingSerial()
    controller = GridController(
        serial,
        clock=clock,
        frame_provider=lambda: frame,
        aruco_detector=OffCenterArucoDetector(),
        base_trim=15,
    )
    plan = build_grid_route(
        "1A",
        GridGeometry(10, 10, 5),
        EncoderCalibration(1, 4, 8),
        turn_source="imu",
    )
    controller.request_grid_mission(plan)
    controller.step_index = 1
    controller._start_current_step()
    controller.step()
    assert serial.commands[-1] == "FORWARD"
    assert any(command.startswith("TRIM=") for command in serial.commands)


def test_aruco_hybrid_route_completes_with_mock_vision():
    serial, clock = FakeSerial(), FakeClock()
    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    class FakeArucoDetector(CenteredArucoDetector):
        pass

    controller = GridController(
        serial,
        clock=clock,
        destination_dwell_seconds=0,
        turn_source="imu",
        frame_provider=lambda: frame,
        aruco_detector=FakeArucoDetector(),
        alignment_confirmation_frames=1,
        aruco_target_area_px=8000.0,
    )
    plan = build_grid_route(
        "1A",
        GridGeometry(10, 10, 5),
        EncoderCalibration(1, 4, 8),
        turn_source="imu",
    )
    controller.request_grid_mission(plan)
    assert controller.get_status()["current_action"] == "ARUCO_ALIGN"

    finish_align_settle(controller, clock)
    controller.step()
    complete_step(controller, serial)
    assert controller.get_status()["current_action"] == "TURN_LEFT"

    serial.turn_status = "DONE"
    for _ in range(80):
        if controller._align_settle_until is not None:
            finish_align_settle(controller, clock)
        controller.step()
        if controller.get_state() == GridState.ARRIVED.value:
            break

    assert controller.get_state() == GridState.ARRIVED.value
