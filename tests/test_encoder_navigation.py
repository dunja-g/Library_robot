import pytest

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
    )
    plan = build_grid_route(
        "1A", GridGeometry(10, 10, 5), EncoderCalibration(1, 4, 8)
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
    assert serial.commands[-1] == "STOP"


def test_reverse_keeps_side_ultrasonic_safety_active():
    controller, serial, clock = make_controller()
    for _ in range(3):
        complete_step(controller, serial)
    clock.now = 1
    controller.step()
    assert controller.get_status()["current_action"] == "BACKWARD"

    serial.ultrasonic["left"] = 10
    controller.step()

    assert controller.get_state() == GridState.STOPPED.value
    assert controller.get_status()["reason"] == "left_obstacle"


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
        "1A", GridGeometry(10, 10, 5), EncoderCalibration(1, 4, 8)
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
        "1A", GridGeometry(10, 10, 5), EncoderCalibration(1, 4, 8)
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
        "2A", GridGeometry(10, 10, 5), EncoderCalibration(1, 4, 8)
    )
    with pytest.raises(RuntimeError, match="already active"):
        controller.request_grid_mission(duplicate)


def test_imu_turn_source_does_not_depend_on_four_tick_encoder_turns():
    serial, clock = FakeSerial(), FakeClock()
    controller = GridController(
        serial, clock=clock, destination_dwell_seconds=0, turn_source="imu"
    )
    plan = build_grid_route(
        "1B", GridGeometry(10, 10, 5), EncoderCalibration(1, 4, 8)
    )
    controller.request_grid_mission(plan)
    complete_step(controller, serial)
    assert controller.get_state() == GridState.TURNING.value
    assert serial.commands[-1] == "TURN_RIGHT"
    serial.turn_status = "DONE"
    controller.step()
    assert controller.get_state() == GridState.MOVING.value
