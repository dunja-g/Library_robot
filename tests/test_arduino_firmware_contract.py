from pathlib import Path


FIRMWARE = (
    Path(__file__).resolve().parents[1]
    / "arduino"
    / "library_robot.ino"
).read_text(encoding="utf-8")


def test_firmware_contains_full_serial_protocol():
    for command in (
        "FORWARD",
        "BACKWARD",
        "ROTATE_LEFT",
        "ROTATE_RIGHT",
        "STOP",
        "CHECK",
        "ENCODER",
        "ENC_RESET",
    ):
        assert f'"{command}"' in FIRMWARE
    assert 'Serial.print("US:")' in FIRMWARE
    assert 'Serial.print("ENC:")' in FIRMWARE
    assert "Serial.begin(115200)" in FIRMWARE


def test_firmware_uses_confirmed_mega_ultrasonic_pins():
    for declaration in (
        "TRIG_LEFT = 25",
        "ECHO_LEFT = 24",
        "TRIG_CENTER = 23",
        "ECHO_CENTER = 22",
        "TRIG_RIGHT = 27",
        "ECHO_RIGHT = 26",
    ):
        assert declaration in FIRMWARE


def test_firmware_has_nonblocking_commands_and_watchdog():
    assert "readStringUntil" not in FIRMWARE
    assert "COMMAND_TIMEOUT_MS = 2000" in FIRMWARE
    assert "motorsActive && millis() - lastCommandMs" in FIRMWARE


def test_firmware_uses_interrupt_encoder_inputs():
    assert "ENCODER_LEFT_PIN = 18" in FIRMWARE
    assert "ENCODER_RIGHT_PIN = 19" in FIRMWARE
    assert "attachInterrupt" in FIRMWARE
    assert "volatile long encoderLeftTicks" in FIRMWARE
