# Fixed 1A-4B Encoder Navigation

This mode removes route-marker dependency for a permanent two-column,
four-row layout. It assumes a centre aisle, column A on the left, column B on
the right, and a repeatable Dock pose facing row 1.

Books use location codes such as `1A-L3-P21`; see `BOOK_NUMBERING.md`. Layer
and position are presented after the base reaches the correct box.

See `HARDWARE_PINOUT.md` for the consolidated encoder, MPU6500, ultrasonic,
and motor-shield wiring.

```text
4A  | aisle |  4B
3A  |       |  3B
2A  |       |  2B
1A  |       |  1B
       Dock
```

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
```

Change `ENCODER_LEFT_PIN` and `ENCODER_RIGHT_PIN` in
`arduino/library_robot.ino` if the final wiring differs, then flash the Mega.

## Measurements still required

Do not enable real grid movement until all six values are measured:

```dotenv
LIBRARY_ROBOT_NAVIGATION_MODE=grid
LIBRARY_ROBOT_GRID_FIRST_ROW_CM=
LIBRARY_ROBOT_GRID_ROW_SPACING_CM=
LIBRARY_ROBOT_GRID_APPROACH_CM=
LIBRARY_ROBOT_ENCODER_TICKS_PER_CM=
LIBRARY_ROBOT_ENCODER_TICKS_PER_REV=4
LIBRARY_ROBOT_WHEEL_DIAMETER_CM=6.5
LIBRARY_ROBOT_ENCODER_TURN_90_TICKS=
LIBRARY_ROBOT_ENCODER_TURN_180_TICKS=
LIBRARY_ROBOT_GRID_TURN_SOURCE=imu
```

- `GRID_FIRST_ROW_CM`: Dock reference line to the turn centre of row 1.
- `GRID_ROW_SPACING_CM`: centre-to-centre distance between adjacent rows.
- `GRID_APPROACH_CM`: centre aisle to the safe stop point at a box.
- `ENCODER_TICKS_PER_CM`: average encoder ticks divided by measured straight
  travel distance.
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
5. Measure left and right 90-degree turns. The current route uses a common
   tick target, so correct mechanical/PWM imbalance before route trials.
6. Fill in the three layout dimensions.
7. Test `1A`, then `1B`, then row 4 to expose accumulated straight-line error.

Keep a person at the power switch during commissioning. The three existing
sensors face the front and sides. They cannot detect an obstacle directly
behind the robot during the configured reverse return.

## Mock test

```powershell
$env:LIBRARY_ROBOT_USE_MOCK="true"
$env:LIBRARY_ROBOT_NAVIGATION_MODE="grid"
python -m pi.app
```

Mock mode uses demonstration-only geometry and encoder values if the six
calibration variables are blank. These values are never used by real mode.
