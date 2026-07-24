from collections import deque

import pytest
import serial

from pi.serial_bridge import SerialBridge, SerialBridgeError


class FakeSerialPort:
    def __init__(self, responses=None, fail_writes=False):
        self.responses = deque(responses or [])
        self.fail_writes = fail_writes
        self.writes = []
        self.flushed = 0
        self.input_reset = False
        self.closed = False

    def write(self, data):
        if self.fail_writes:
            raise serial.SerialException("write failed")
        self.writes.append(data)

    def flush(self):
        self.flushed += 1

    def readline(self):
        return self.responses.popleft() if self.responses else b""

    def reset_input_buffer(self):
        self.input_reset = True

    def close(self):
        self.closed = True


def make_bridge(fake):
    return SerialBridge(
        port="FAKE",
        startup_delay=0,
        serial_factory=lambda **_kwargs: fake,
    )


def test_motion_commands_match_arduino_protocol():
    fake = FakeSerialPort()
    bridge = make_bridge(fake)

    bridge.send_forward()
    bridge.send_backward()
    bridge.send_rotate_left()
    bridge.send_rotate_right()
    bridge.send_turn_left()
    bridge.send_turn_right()
    bridge.send_turn_uturn()
    bridge.send_stop()

    assert fake.writes == [
        b"FORWARD\n",
        b"BACKWARD\n",
        b"ROTATE_LEFT\n",
        b"ROTATE_RIGHT\n",
        b"TURN_LEFT\n",
        b"TURN_RIGHT\n",
        b"TURN_UTURN\n",
        b"STOP\n",
    ]
    assert fake.input_reset


def test_bridge_exposes_every_method_required_by_robot_controller():
    bridge = make_bridge(FakeSerialPort())
    for method_name in (
        "send_forward",
        "send_rotate_left",
        "send_rotate_right",
        "send_turn_left",
        "send_turn_right",
        "send_turn_uturn",
        "send_stop",
        "get_ultrasonic",
    ):
        assert callable(getattr(bridge, method_name))


def test_ultrasonic_ignores_status_line_and_parses_three_values():
    fake = FakeSerialPort([b"READY\r\n", b"US:43.2,18.7,55.1\r\n"])
    bridge = make_bridge(fake)

    readings = bridge.get_ultrasonic()

    assert fake.writes == [b"CHECK\n"]
    assert readings == {"left": 43.2, "center": 18.7, "right": 55.1}


@pytest.mark.parametrize(
    "line",
    [
        "",
        "READY",
        "US:1,2",
        "US:1,two,3",
        "US:-1,2,3",
        "US:nan,2,3",
        "US:inf,2,3",
    ],
)
def test_ultrasonic_parser_rejects_invalid_or_unsafe_values(line):
    assert SerialBridge.parse_ultrasonic(line) is None


def test_encoder_reset_and_read_protocol():
    fake = FakeSerialPort([b"ENC_RESET:OK\r\n", b"ENC:120,118\r\n"])
    bridge = make_bridge(fake)
    assert bridge.reset_encoders() is True
    assert bridge.get_encoders() == {"left": 120, "right": 118}
    assert fake.writes == [b"ENC_RESET\n", b"ENCODER\n"]


def test_fused_odometry_configuration_and_telemetry_protocol():
    fake = FakeSerialPort(
        [b"FUSION:OK\r\n", b"ODOM:12,11,6.0,5.5,-2.1,-1.8,-1.815,4\r\n"]
    )
    bridge = make_bridge(fake)

    assert bridge.set_fusion_config(
        alpha=0.95,
        left_ticks_per_cm=2.0,
        right_ticks_per_cm=2.0,
        wheel_track_cm=14.5,
        heading_kp=2.0,
        max_correction=40,
    )
    assert bridge.get_odometry() == {
        "left": 12,
        "right": 11,
        "left_cm": 6.0,
        "right_cm": 5.5,
        "heading_encoder_deg": -2.1,
        "heading_imu_deg": -1.8,
        "heading_fused_deg": -1.815,
        "speed_correction": 4,
    }
    assert fake.writes == [
        b"SET_FUSION:0.9500,2.000000,2.000000,14.500,2.000,40\n",
        b"ODOMETRY\n",
    ]


@pytest.mark.parametrize(
    "line",
    ["", "ODOM:1,2", "ODOM:1,2,3,4,5,6,nan,8", "ODOM:a,2,3,4,5,6,7,8"],
)
def test_odometry_parser_rejects_invalid_data(line):
    assert SerialBridge.parse_odometry(line) is None


@pytest.mark.parametrize("line", ["", "ENC:1", "ENC:1,two", "ENC:1,2,3"])
def test_encoder_parser_rejects_invalid_data(line):
    assert SerialBridge.parse_encoders(line) is None


def test_turn_status_protocol():
    fake = FakeSerialPort([b"TURN_DONE:90.2\r\n", b"TURN:DONE\r\n"])
    bridge = make_bridge(fake)
    assert bridge.get_turn_status() == "DONE"
    assert fake.writes == [b"TURN_STATUS\n"]


@pytest.mark.parametrize("line", ["", "TURN:MAYBE", "TURN_DONE:90"])
def test_turn_status_parser_rejects_invalid_data(line):
    assert SerialBridge.parse_turn_status(line) is None


def test_write_failure_is_reported_without_crashing_controller():
    bridge = make_bridge(FakeSerialPort(fail_writes=True))
    assert bridge.send_forward() is False
    assert bridge.get_ultrasonic() is None


def test_close_sends_stop_and_is_idempotent():
    fake = FakeSerialPort()
    bridge = make_bridge(fake)

    bridge.close()
    bridge.close()

    assert fake.writes == [b"STOP\n"]
    assert fake.closed
    assert bridge.send_forward() is False


def test_open_failure_has_clear_domain_error():
    def failing_factory(**_kwargs):
        raise serial.SerialException("port missing")

    with pytest.raises(SerialBridgeError, match="Unable to open Arduino"):
        SerialBridge(startup_delay=0, serial_factory=failing_factory)
