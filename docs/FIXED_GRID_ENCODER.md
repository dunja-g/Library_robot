# Fixed 1A-3B Fused-Odometry Navigation

This mode removes route-marker dependency for a permanent two-column,
three-row layout. It assumes a centre aisle, column A on the left, column B on
the right, and a repeatable Dock pose facing row 1.

Books use location codes such as `1A-L3-P21`; see `BOOK_NUMBERING.md`. Layer
and position are presented after the base reaches the correct box.

See `HARDWARE_PINOUT.md` for the consolidated encoder, MPU6500, ultrasonic,
and motor-shield wiring.

```text
3A  |       |  3B
2A  |       |  2B
1A  |       |  1B
       Dock
```

## Sensor fusion and straight-line control

Every motion segment starts with `ENC_RESET`. The Mega converts the two
counters independently:

```text
d_left  = signed_left_ticks  / left_ticks_per_cm
d_right = signed_right_ticks / right_ticks_per_cm
distance = (d_left + d_right) / 2
theta_encoder = (d_right - d_left) / wheel_track
```

Encoder signs come from the commanded wheel directions, so the same equations
also cover reverse and in-place turns with the installed single-channel pulse
encoders. The MPU6500 Z gyroscope is integrated continuously during forward,
backward, left-turn and right-turn motion:

```text
theta_imu(t) = theta_imu(t-1) + (gyro_z - bias) * dt
theta_fused = alpha * theta_imu + (1 - alpha) * theta_encoder
```

Start with `alpha=0.95`. During a straight segment the Mega applies a bounded
proportional correction from `theta_fused` to the left and right PWM values.
The correction polarity is reversed automatically for backward travel. This
loop runs on the Mega independently of Flask request or camera frame rate.

## Route generation

For a destination such as `3B`, the generated route is:

1. Drive from Dock to row 3 using encoder distance.
2. Turn right by the calibrated 90-degree tick count.
3. Approach the box using encoder distance.
4. Stop, show the exact layer/position, and wait for pickup confirmation.
5. Reverse the approach distance back to the centre aisle.
6. Apply the mirrored 90-degree turn to align with the aisle.
7. Reverse the measured row distance back and enter `DOCKED`.

The A route mirrors the B route. Route execution is non-blocking. All three
ultrasonic data is validated during every moving or turning iteration.
Missing encoder data, no encoder progress, missing ultrasonic data, or an
obstacle causes `STOPPED`.

The return route is a space-constrained reverse route. During reverse, the
left/right ultrasonic thresholds remain active; the front-centre reading is
ignored because it faces the shelf the robot is leaving. There is no
rear-facing obstacle sensor, so the reverse corridor must be cleared and
supervised.

## Mega encoder wiring

The firmware currently defines:

```text
Left encoder pulse  -> Mega pin 18
Right encoder pulse -> Mega pin 19
Signal ground       -> Mega GND
```

Pins 18 and 19 are interrupt-capable on the Mega. Confirm the encoder voltage
is safe for the Mega before connection. The initial firmware counts one
`RISING` pulse channel per side. If the installed encoders are quadrature
encoders, connect channel A first; channel B can later be used for independent
direction validation.

The serial protocol adds:

```text
ENC_RESET  -> reset counters; Mega replies ENC_RESET:OK
ENCODER    -> Mega replies ENC:left,right
ODOMETRY   -> Mega replies ODOM:left,right,d_left,d_right,theta_enc,theta_imu,theta_fused,correction
SET_FUSION -> configure scales, wheel track, alpha and closed-loop gain
```

Change `ENCODER_LEFT_PIN` and `ENCODER_RIGHT_PIN` in
`arduino/library_robot.ino` if the final wiring differs, then flash the Mega.

## Measurements still required

Do not enable real grid movement until the layout and fused odometry are measured:

```dotenv
LIBRARY_ROBOT_NAVIGATION_MODE=grid
LIBRARY_ROBOT_GRID_FIRST_ROW_CM=
LIBRARY_ROBOT_GRID_ROW_SPACING_CM=48
LIBRARY_ROBOT_GRID_APPROACH_CM=15
LIBRARY_ROBOT_ENCODER_TICKS_PER_CM=
LIBRARY_ROBOT_ENCODER_TICKS_PER_REV=4
LIBRARY_ROBOT_WHEEL_DIAMETER_CM=6.5
LIBRARY_ROBOT_ENCODER_TURN_90_TICKS=
LIBRARY_ROBOT_ENCODER_TURN_180_TICKS=
LIBRARY_ROBOT_GRID_TURN_SOURCE=imu
LIBRARY_ROBOT_GRID_LINEAR_SOURCE=encoder
LIBRARY_ROBOT_AUTO_RETURN=true
LIBRARY_ROBOT_LEFT_TICKS_PER_CM=0.195883
LIBRARY_ROBOT_RIGHT_TICKS_PER_CM=0.195883
LIBRARY_ROBOT_WHEEL_TRACK_CM=18.0
LIBRARY_ROBOT_FUSION_ALPHA=0.95
LIBRARY_ROBOT_HEADING_KP=1.5
LIBRARY_ROBOT_MAX_HEADING_CORRECTION=30
```

- `GRID_FIRST_ROW_CM`: Dock reference line to the turn centre of row 1.
- `GRID_ROW_SPACING_CM`: centre-to-centre distance between adjacent rows.
- `GRID_APPROACH_CM`: centre aisle to the safe stop point at a box.
- `GRID_LINEAR_SOURCE=encoder`: straight outbound and reverse-return segments
  stop from encoder distance. Select `timed` only for deliberate bench tests.
- `AUTO_RETURN=true`: after the configured destination dwell, begin the
  direct-reverse return automatically. Set it to `false` to require the UI
  pickup confirmation button.
- `LEFT/RIGHT_TICKS_PER_CM`: calibrate each wheel independently.
- `WHEEL_TRACK_CM`: centre-to-centre distance between the left and right wheels.
- `FUSION_ALPHA`: IMU weight in the complementary filter; start at `0.95`.
- The committed `18.0 cm` wheel track is a photo-based initial estimate. It
  must be replaced by the measured centre-to-centre tyre distance.
- Alternatively, leave ticks-per-cm blank and provide the confirmed
  `ENCODER_TICKS_PER_REV=4` plus measured wheel diameter.
- `ENCODER_TURN_90_TICKS`: average ticks during a physical 90-degree turn.
- `ENCODER_TURN_180_TICKS`: independently measured 180-degree turn ticks.

When `GRID_TURN_SOURCE=imu`, the two turn-tick values may remain blank because
turn completion comes from the MPU6500. This is recommended for the current
four-count-per-revolution encoder.

The server reports the missing values at `/navigation_mode` and refuses
`/request_box` with HTTP 503 until calibration is complete.

## Calibration order

1. Raise the wheels and verify both encoder counters increase through
   `ENCODER`.
2. Reset with `ENC_RESET` and verify both return to zero.
3. Place the robot in a repeatable mechanical Dock guide.
4. Drive straight for a measured distance at least five times and calculate
   ticks per centimetre from the average.
5. Measure the wheel track, then verify encoder and IMU heading signs agree.
6. Fill in the three layout dimensions.
7. Test `1A`, then `1B`, then row 3 to expose accumulated straight-line error.

Keep a person at the power switch during commissioning. The three existing
sensors face the front and sides. They cannot detect an obstacle directly
behind the robot during the configured reverse return.

`pi/app.py` runs `GridController` from `pi/encoder_navigation.py`.
`pi/robot_controller.py` is the archived ArUco waypoint controller and is not
part of the fixed-grid Flask runtime.

## Mock test

```powershell
$env:LIBRARY_ROBOT_USE_MOCK="true"
$env:LIBRARY_ROBOT_NAVIGATION_MODE="grid"
python -m pi.app
```

Mock mode uses demonstration-only geometry and encoder values if the six
calibration variables are blank. These values are never used by real mode.
