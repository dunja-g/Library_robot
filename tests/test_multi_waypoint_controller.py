import numpy as np

from pi.book_db import get_book
from pi.robot_controller import RobotController, State


class FakeClock:
    def __init__(self): self.now = 0.0
    def __call__(self): return self.now


class FakeCamera:
    def get_frame(self): return np.zeros((480, 640, 3), dtype=np.uint8)


class FakeDetector:
    def __init__(self): self.detections = []
    def detect(self, _frame): return self.detections
    def draw(self, frame, _detections): return frame.copy()


class FakeSerial:
    def __init__(self):
        self.commands = []
        self.readings = {"left": 100, "center": 100, "right": 100}
    def send_forward(self): self.commands.append("FORWARD")
    def send_rotate_left(self): self.commands.append("ROTATE_LEFT")
    def send_rotate_right(self): self.commands.append("ROTATE_RIGHT")
    def send_stop(self): self.commands.append("STOP")
    def get_ultrasonic(self): return self.readings


def detection(marker_id, center_x=320):
    return {"id": marker_id, "center_x": center_x, "center_y": 240,
            "area": 1000.0, "corners": np.zeros((4, 2))}


def make_controller():
    serial, detector, clock = FakeSerial(), FakeDetector(), FakeClock()
    controller = RobotController(
        serial, FakeCamera(), detector, clock=clock,
        target_confirmation_frames=1, alignment_confirmation_frames=1,
        turn_90_seconds=0.8, uturn_seconds=1.6,
        destination_dwell_seconds=2.0,
    )
    controller.request_mission(get_book("Deep Learning"))
    return controller, serial, detector, clock


def reach_current_marker(controller, serial, detector, marker_id, distance):
    serial.readings["center"] = 100
    detector.detections = [detection(marker_id)]
    controller.step()
    assert controller.get_state() == State.ALIGNING.value
    controller.step()
    assert controller.get_state() == State.APPROACHING.value
    serial.readings["center"] = distance
    controller.step()


def finish_turn(controller, serial, clock, duration):
    serial.readings["center"] = 100
    before = clock.now
    controller.step()
    assert clock.now == before  # Controller never sleeps or mutates the clock.
    clock.now += duration
    controller.step()
    assert controller.get_state() == State.SCANNING.value


def test_complete_outbound_dwell_return_and_dock_route():
    controller, serial, detector, clock = make_controller()
    assert controller.get_status()["route"] == [101, 105, 203]

    reach_current_marker(controller, serial, detector, 101, 34)
    assert controller.get_state() == State.TURNING.value
    assert controller.get_status()["current_marker_id"] == 105
    finish_turn(controller, serial, clock, 0.8)

    reach_current_marker(controller, serial, detector, 105, 34)
    assert controller.get_state() == State.TURNING.value
    finish_turn(controller, serial, clock, 0.8)

    reach_current_marker(controller, serial, detector, 203, 34)
    assert controller.get_state() == State.ARRIVED.value
    assert controller.get_status()["phase"] == "AT_DESTINATION"

    clock.now += 2.0
    controller.step()
    assert controller.get_state() == State.TURNING.value
    assert controller.get_status()["phase"] == "RETURNING"
    assert controller.get_status()["current_marker_id"] == 105
    finish_turn(controller, serial, clock, 1.6)

    reach_current_marker(controller, serial, detector, 105, 34)
    finish_turn(controller, serial, clock, 0.8)
    reach_current_marker(controller, serial, detector, 101, 34)
    finish_turn(controller, serial, clock, 0.8)
    reach_current_marker(controller, serial, detector, 0, 29)

    status = controller.get_status()
    assert status["state"] == State.DOCKED.value
    assert status["phase"] == "COMPLETE"
    assert status["reason"] == "dock_reached"


def test_wrong_marker_never_advances_waypoint():
    controller, serial, detector, _clock = make_controller()
    detector.detections = [detection(105)]
    controller.step()
    assert controller.get_state() == State.SCANNING.value
    assert controller.get_status()["current_marker_id"] == 101
    assert serial.commands[-1] == "ROTATE_LEFT"


def test_close_obstacle_without_visible_target_is_not_arrival():
    controller, serial, detector, _clock = make_controller()
    controller.state = State.APPROACHING
    detector.detections = [detection(105)]
    serial.readings["center"] = 10
    controller.step()
    assert controller.get_state() == State.STOPPED.value
    assert controller.get_status()["reason"] == "front_obstacle"
    assert controller.get_status()["current_marker_id"] == 101


def test_reset_clears_mission_and_timed_turn_state():
    controller, serial, detector, _clock = make_controller()
    reach_current_marker(controller, serial, detector, 101, 34)
    assert controller.get_state() == State.TURNING.value
    controller.reset()
    assert controller.get_state() == State.IDLE.value
    assert controller.mission is None
    assert controller._turn_deadline is None
    assert controller.get_status()["target_id"] is None
