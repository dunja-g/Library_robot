import numpy as np

from pi.robot_controller import RobotController, State


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class FakeCamera:
    def get_frame(self):
        return np.zeros((480, 640, 3), dtype=np.uint8)


class FakeDetector:
    def __init__(self):
        self.detections = []

    def detect(self, _frame):
        return self.detections

    def draw(self, frame, _detections):
        return frame.copy()


class FakeSerial:
    def __init__(self):
        self.commands = []
        self.readings = {"left": 100, "center": 100, "right": 100}

    def send_forward(self):
        self.commands.append("FORWARD")

    def send_rotate_left(self):
        self.commands.append("ROTATE_LEFT")

    def send_rotate_right(self):
        self.commands.append("ROTATE_RIGHT")

    def send_stop(self):
        self.commands.append("STOP")

    def get_ultrasonic(self):
        return self.readings


def detection(marker_id=1, center_x=320):
    return {
        "id": marker_id,
        "center_x": center_x,
        "center_y": 240,
        "area": 1000.0,
        "corners": np.array([[300, 220], [340, 220], [340, 260], [300, 260]]),
    }


def make_controller():
    serial = FakeSerial()
    detector = FakeDetector()
    clock = FakeClock()
    controller = RobotController(
        serial,
        FakeCamera(),
        detector,
        clock=clock,
        scan_timeout_seconds=10,
    )
    return controller, serial, detector, clock


def test_complete_nominal_transition_to_arrived():
    controller, serial, detector, _clock = make_controller()
    controller.request_book(1)
    assert controller.get_state() == State.SCANNING.value

    detector.detections = [detection(1, 320)]
    controller.step()
    assert controller.get_state() == State.ALIGNING.value

    controller.step()
    assert controller.get_state() == State.APPROACHING.value

    serial.readings["center"] = 24
    controller.step()
    assert controller.get_state() == State.ARRIVED.value
    assert serial.commands[-1] == "STOP"


def test_scans_until_requested_marker_is_seen():
    controller, serial, detector, _clock = make_controller()
    controller.request_book(2)
    detector.detections = [detection(1)]
    controller.step()

    assert controller.get_state() == State.SCANNING.value
    assert serial.commands[-1] == "ROTATE_LEFT"


def test_alignment_turns_in_direction_of_marker_error():
    controller, serial, detector, _clock = make_controller()
    controller.request_book(1)
    controller.state = State.ALIGNING

    detector.detections = [detection(1, 200)]
    controller.step()
    assert serial.commands[-1] == "ROTATE_LEFT"

    detector.detections = [detection(1, 450)]
    controller.step()
    assert serial.commands[-1] == "ROTATE_RIGHT"


def test_target_loss_during_approach_stops_before_scanning():
    controller, serial, _detector, _clock = make_controller()
    controller.request_book(1)
    controller.state = State.APPROACHING
    controller.step()

    assert controller.get_state() == State.SCANNING.value
    assert serial.commands[-1] == "STOP"


def test_side_obstacle_triggers_stopped():
    controller, serial, detector, _clock = make_controller()
    controller.request_book(1)
    controller.state = State.APPROACHING
    detector.detections = [detection()]
    serial.readings["left"] = 10
    controller.step()

    assert controller.get_status()["reason"] == "side_obstacle"
    assert controller.get_state() == State.STOPPED.value


def test_invalid_ultrasonic_data_fails_safe():
    controller, serial, detector, _clock = make_controller()
    controller.request_book(1)
    controller.state = State.APPROACHING
    detector.detections = [detection()]
    serial.readings = None
    controller.step()

    assert controller.get_state() == State.STOPPED.value
    assert controller.get_status()["reason"] == "ultrasonic_unavailable"


def test_scan_timeout_stops_robot():
    controller, serial, _detector, clock = make_controller()
    controller.request_book(1)
    clock.now = 11
    controller.step()

    assert controller.get_state() == State.STOPPED.value
    assert controller.get_status()["reason"] == "scan_timeout"
    assert serial.commands[-1] == "STOP"


def test_reset_stops_and_clears_target_from_any_state():
    controller, serial, _detector, _clock = make_controller()
    controller.request_book(1)
    controller.state = State.APPROACHING
    controller.reset()

    assert controller.get_state() == State.IDLE.value
    assert controller.get_status()["target_id"] is None
    assert serial.commands[-1] == "STOP"


def test_latest_frame_is_a_defensive_copy():
    controller, _serial, _detector, _clock = make_controller()
    controller.step()
    first = controller.get_latest_frame()
    first[:] = 255

    assert not np.array_equal(first, controller.get_latest_frame())
