# Final Hardware Pinout

The canonical firmware is `arduino/library_robot.ino`. The canonical Raspberry
Pi bridge is `pi/serial_bridge.py`.

The Raspberry Pi 5 communicates with the Mega over USB serial. Encoder pulse
signals must go to the Mega, not directly to Raspberry Pi GPIO: the Mega
provides deterministic interrupt counting and electrical isolation from Linux
timing jitter.

## Arduino Mega connections

| Function | Mega pin | Notes |
|---|---:|---|
| Left encoder pulse / channel A | 18 | Interrupt-capable; also TX1, so do not use Serial1 |
| Right encoder pulse / channel A | 19 | Interrupt-capable; also RX1, so do not use Serial1 |
| MPU6500 SDA | 20 | Mega I2C SDA |
| MPU6500 SCL | 21 | Mega I2C SCL |
| Front ultrasonic ECHO | 22 | HC-SR04 front distance sensor |
| Front ultrasonic TRIG | 23 | HC-SR04 front distance sensor |

The AFMotor V1 shield uses motor channels M1/M4 for the left side and M2/M3
for the right side. Its control/PWM pins do not overlap Mega pins 18-23.
USB serial uses Mega pins 0/1 internally, so encoder pins 18/19 do not conflict
with the normal USB connection.

## Encoder wiring

For the current one-pulse-per-side implementation:

```text
Left encoder A/SIGNAL  -> Mega D18
Right encoder A/SIGNAL -> Mega D19
Left encoder GND       -> Mega GND
Right encoder GND      -> Mega GND
Encoder B channels     -> leave disconnected and insulated for this version
```

Connect exactly one representative encoder per drivetrain side. If all four
motors have encoders, use one left and one right encoder initially; four-wheel
counting requires another firmware revision.

The encoder supply connection cannot be finalised without the encoder model:

- Connect VCC only to the voltage specified by the encoder datasheet/module.
- Never assume a bare 3.3 V sensor accepts 5 V.
- The firmware enables the Mega's internal pull-up on both signal inputs.
- An open-collector output may need an external 4.7-10 kOhm pull-up to its
  permitted logic voltage if the internal pull-up is too weak at speed.
- All grounds (Mega, motor supply reference, encoder modules, ultrasonic
  modules, and IMU) must share a common reference. Do not power motors from
  the Mega 5 V pin.

If the encoder has only `VCC/GND/OUT`, `OUT` is the pulse signal. If it has
`VCC/GND/A/B`, use channel A now. Send a photo or model number before applying
power if its voltage or pin labels are unclear.

## MPU6500 wiring

```text
MPU6500 SDA -> Mega D20
MPU6500 SCL -> Mega D21
MPU6500 GND -> Mega GND
MPU6500 VCC -> voltage required by the specific breakout board
```

Many MPU6500 breakout boards include regulation and level shifting, but bare
modules may be 3.3 V only. Check the board marking or datasheet before choosing
3.3 V or 5 V.

## Integrated serial commands

| Command | Purpose |
|---|---|
| `FORWARD`, `BACKWARD` | Continuous straight movement |
| `ROTATE_LEFT`, `ROTATE_RIGHT` | Continuous rotation for camera/encoder control |
| `TURN_LEFT`, `TURN_RIGHT` | Non-blocking MPU6500 90-degree turn |
| `TURN_UTURN` | Non-blocking MPU6500 180-degree left turn |
| `TURN_STATUS` | Return `TURN:IDLE/ACTIVE/DONE/ERROR` |
| `ENC_RESET` | Reset left/right encoder counters |
| `ODOMETRY` | Read wheel distances, encoder/IMU/fused heading and PWM correction |
| `SET_FUSION:...` | Configure complementary fusion and straight-line feedback |
| `ENCODER` | Return `ENC:left,right` |
| `CHECK` | Return `US:left,center,right` |
| `STOP` | Cancel any turn and stop all motors |

The integrated IMU turn implementation does not block the serial parser.
An IMU turn that cannot reach its angle within five seconds stops with
`TURN_ERROR:TIMEOUT`.

## Confirmed encoder resolution

The measured resolution is currently **4 counts per wheel revolution** using
one rising-edge channel. This is valid but coarse. Configure:

```dotenv
LIBRARY_ROBOT_ENCODER_TICKS_PER_REV=4
LIBRARY_ROBOT_WHEEL_DIAMETER_CM=6.5
LIBRARY_ROBOT_GRID_TURN_SOURCE=imu
```

With only four counts, one distance tick represents one quarter of the wheel
circumference. The confirmed 6.5 cm wheel moves about 5.1 cm per tick.
The grid controller therefore uses the MPU6500 for turns by default and
reserves encoder counts for straight distance and stall detection.

## First powered verification

1. Keep the chassis wheels raised.
2. Power the Mega and wait for `IMU:READY`.
3. Send `ENC_RESET`, then rotate the left wheel by hand and request `ENCODER`.
   Only the left count should increase.
4. Repeat for the right wheel.
5. Send `FORWARD` briefly and verify both counts increase.
6. Send `STOP` before placing the chassis on the floor.
7. Test `TURN_LEFT` and `TURN_RIGHT` with space around the chassis.
8. Verify `CHECK` still returns all three ultrasonic readings.
