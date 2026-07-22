# Person 1 — Hardware & Communication Guide

## Your Role
You are responsible for all low-level hardware: the Arduino firmware that drives the motors and reads the ultrasonic sensors, and the Python serial bridge that lets the Raspberry Pi talk to the Arduino cleanly.

**Your files:**
- `arduino/library_robot.ino`
- `pi/serial_bridge.py`

---

## What You Need

### Hardware
- Arduino Mega
- MH Electronics Motor Shield (AFMotor library)
- 4× DC motors connected to M1, M2, M3, M4 on the shield
- 3× HC-SR04 ultrasonic sensors (Left, Centre, Right)
- USB cable (Arduino → Raspberry Pi)

### Software (on your PC to flash Arduino)
- Arduino IDE: https://www.arduino.cc/en/software
- **Adafruit Motor Shield V1** library (install via Arduino IDE → Sketch → Include Library → Manage Libraries → search "Adafruit Motor Shield")

---

## Step 1 — Understand the Serial Protocol

The Raspberry Pi will send plain text commands to the Arduino over USB Serial. The Arduino must respond correctly. This is the agreed interface with Person 2:

| Pi sends | Arduino does |
|---|---|
| `ROTATE_LEFT\n` | Spin left in place (slow speed ~120/255) |
| `ROTATE_RIGHT\n` | Spin right in place (slow speed ~120/255) |
| `FORWARD\n` | Drive all 4 wheels forward (speed ~180/255) |
| `STOP\n` | Stop all motors |
| `CHECK\n` | Read 3 ultrasonic sensors, reply with `US:L,C,R\n` |

**Example Arduino reply:**
```
US:43.2,18.7,55.1
```

---

## Step 2 — Wire the Ultrasonic Sensors

Connect your 3× HC-SR04 sensors to the confirmed Arduino Mega pins:

| Sensor | TRIG pin | ECHO pin |
|---|---|---|
| Left | 25 | 24 |
| Centre | 23 | 22 |
| Right | 27 | 26 |

> **Note:** HC-SR04 uses 5V. Make sure TRIG and ECHO are connected to 5V-tolerant digital pins.

---

## Step 3 — Write `arduino/library_robot.ino`

Create the file `arduino/library_robot.ino`. It must do the following:

```
Setup:
  - Start Serial at 115200 baud
  - Configure ultrasonic pins
  - Initialise AFMotor objects for M1, M2, M3, M4

Loop:
  - Check for incoming serial commands
  - If command == "ROTATE_LEFT"  → left motors backward, right motors forward (slow)
  - If command == "ROTATE_RIGHT" → left motors forward, right motors backward (slow)
  - If command == "FORWARD"      → all motors forward (moderate speed)
  - If command == "STOP"         → release all motors
  - If command == "CHECK"        → read 3 ultrasonic sensors, print "US:L,C,R"
  - Safety timeout: if no command received for 2 seconds → auto STOP
```

**Motor layout:**
- Left side:  M1 (front-left) + M4 (rear-left)
- Right side: M2 (front-right) + M3 (rear-right)

**Rotate Left** (spin in place):
- Left motors → BACKWARD
- Right motors → FORWARD

**Rotate Right** (spin in place):
- Left motors → FORWARD
- Right motors → BACKWARD

**Ultrasonic distance formula:**
```cpp
float readDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH, 30000);
    if (duration == 0) return 999.0; // no echo = out of range
    return duration * 0.0343 / 2.0;  // convert to cm
}
```

---

## Step 4 — Test the Arduino Independently

1. Flash `library_robot.ino` to the Arduino using Arduino IDE.
2. Open **Serial Monitor** (Tools → Serial Monitor, set baud rate to **115200**).
3. Type `FORWARD` and press Enter → all 4 wheels should spin forward.
4. Type `STOP` → wheels stop.
5. Type `ROTATE_LEFT` → robot spins left in place.
6. Type `CHECK` → you should get back something like `US:34.5,22.1,41.8`.
7. Hold your hand ~10cm in front of the centre sensor and type `CHECK` again → centre value should be small (~10).

✅ **Pass criteria:** All 5 commands work correctly in Serial Monitor.

---

## Step 5 — Write `pi/serial_bridge.py`

This is the Python module that Person 2's navigation code will call. It wraps the raw serial communication into clean, easy-to-use functions.

**Required functions:**

```python
class SerialBridge:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        # Connect to Arduino over USB serial
        # Wait 2 seconds for Arduino to reset after connection
        pass

    def send_forward(self):
        # Send "FORWARD\n" to Arduino
        pass

    def send_rotate_left(self):
        # Send "ROTATE_LEFT\n" to Arduino
        pass

    def send_rotate_right(self):
        # Send "ROTATE_RIGHT\n" to Arduino
        pass

    def send_stop(self):
        # Send "STOP\n" to Arduino
        pass

    def get_ultrasonic(self):
        # Send "CHECK\n" to Arduino
        # Read the response line
        # Parse "US:43.2,18.7,55.1" into a dict:
        # Returns: {"left": 43.2, "center": 18.7, "right": 55.1}
        # Returns None if parsing fails
        pass

    def close(self):
        # Close the serial port cleanly
        pass
```

**Important notes:**
- Use `pyserial`: `import serial`
- Set `timeout=0.5` on the serial connection so `readline()` doesn't hang forever
- Strip whitespace/newlines from responses before parsing: `line.strip()`
- Wrap everything in try/except and log errors — don't let a serial hiccup crash the whole robot

---

## Step 6 — Test `serial_bridge.py` Independently

Create a quick test script `pi/test_serial.py`:

```python
from serial_bridge import SerialBridge
import time

bridge = SerialBridge(port='/dev/ttyACM0')

print("Testing FORWARD...")
bridge.send_forward()
time.sleep(1)
bridge.send_stop()

print("Testing ROTATE_LEFT...")
bridge.send_rotate_left()
time.sleep(1)
bridge.send_stop()

print("Testing ultrasonic...")
readings = bridge.get_ultrasonic()
print(f"Ultrasonic: {readings}")

bridge.close()
print("Done!")
```

Run this on the Pi with the Arduino connected: `python pi/test_serial.py`

✅ **Pass criteria:** Motors behave correctly and ultrasonic readings print as a dictionary.

---

## Handoff to Team

Once you are done:
1. Commit and push `arduino/library_robot.ino` and `pi/serial_bridge.py`
2. Tell Person 2 that the `SerialBridge` class is ready and share the exact serial port name (e.g., `/dev/ttyACM0`)
3. Tell Person 3 what port the Arduino is on, so they can put it in the config

---

## Common Issues

| Problem | Fix |
|---|---|
| `No module named 'serial'` | Run `pip install pyserial` on the Pi |
| `Permission denied /dev/ttyACM0` | Run `sudo usermod -a -G dialout $USER` then reboot |
| Motors spin in wrong direction | Swap the wiring on that motor's terminals on the shield, OR flip `FORWARD`/`BACKWARD` in code for that motor |
| `CHECK` returns nothing | Make sure baud rate in Serial Monitor AND in `serial_bridge.py` are both `115200` |
