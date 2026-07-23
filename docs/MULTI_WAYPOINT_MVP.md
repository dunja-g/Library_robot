# Multi-Waypoint MVP Runbook

## Demonstration layout

Use `DICT_4X4_50` ArUco markers matching the detector configuration. Print markers flat, with a quiet white border, and large enough for reliable detection at the intended approach distance.

| Marker | Placement | Robot action after verified arrival |
|---:|---|---|
| 0 | Dock, visible on the final return approach | Stop and enter `DOCKED` |
| 101 | Main corridor waypoint | Turn right |
| 105 | Zone B junction | Outbound: turn left; return: turn right |
| 203 | Shelf B3, adjacent to *Deep Learning* | Stop, dwell, begin return with a U-turn |

The outbound path is `101 → 105 → 203`. The return path is `105 → 101 → 0`. Place each marker facing the robot's expected incoming direction. Timed turns assume the next marker becomes visible after the turn; adjust marker angle or turn duration if it does not.

## Configuration

All values are read once when the real-hardware server starts.

| Environment variable | Default | Tune on hardware |
|---|---:|---|
| `LIBRARY_ROBOT_ALIGN_TOLERANCE_PX` | 30 | Raise slightly if alignment chatters; lower for tighter centring |
| `LIBRARY_ROBOT_OBSTACLE_DISTANCE_CM` | 20 | Emergency clearance, based on chassis stopping distance |
| `LIBRARY_ROBOT_STOP_DISTANCE_CM` | 35 | Fallback only; configured markers have their own arrival distance |
| `LIBRARY_ROBOT_TURN_90_SECONDS` | 0.8 | Duration of LEFT/RIGHT turns |
| `LIBRARY_ROBOT_UTURN_SECONDS` | 1.6 | Duration of the destination U-turn |
| `LIBRARY_ROBOT_DESTINATION_DWELL_SECONDS` | 5 | Time the UI remains at `ARRIVED` before return |
| `LIBRARY_ROBOT_AUTO_RETURN` | true | Set false to remain at the shelf |
| `LIBRARY_ROBOT_SCAN_TIMEOUT_SECONDS` | 60 | Maximum search time for each waypoint |

Tune turns at low motor speed in an open area. Start with the wheels raised, verify LEFT and RIGHT direction, then measure the time for a repeatable 90° rotation on the actual floor. Set U-turn independently; do not assume it is exactly twice the 90° value. Repeat with the battery at typical charge.

Marker-specific arrival distances are in `pi/route_db.py`: 35 cm for route waypoints and shelf B3, 30 cm for Dock. Keep the obstacle threshold below each arrival threshold so a visible, centred expected marker can be accepted before the emergency clearance is crossed.

## Safety decision order

Every active navigation state requests all three HC-SR04 readings. Invalid or missing data stops the robot. During approach, the controller applies this order:

1. Reject invalid ultrasonic data.
2. Stop for a left or right obstacle.
3. Find the exact expected ArUco ID; another marker cannot advance the route.
4. If the expected marker is missing or misaligned and the centre range is inside emergency clearance, stop as `front_obstacle`.
5. Declare a waypoint reached only when the expected marker is visible, centred, and the centre range is at or below that marker's arrival distance.
6. Otherwise drive forward.

Timed turns use monotonic deadlines and never sleep inside the controller. Ultrasonic safety remains active while turning.

## Mock demonstration

From the repository root:

```powershell
$env:LIBRARY_ROBOT_USE_MOCK="true"
python -m pi.app
```

Select **Deep Learning**. Mock mode progresses through all three outbound markers, announces arrival in browsers supporting Web Speech, simulates the return route, and announces `DOCKED`. It exercises mission and UI behavior, not camera geometry, serial timing, motor traction, or sonar accuracy.

## Raspberry Pi commissioning

1. Confirm Mega commands individually: `STOP`, `FORWARD`, `ROTATE_LEFT`, `ROTATE_RIGHT`, and `CHECK`.
2. Verify `CHECK` returns finite left/centre/right distances and deliberately unplug one sensor to confirm fail-safe stopping.
3. Verify the camera detects each printed ID under library lighting.
4. Put the chassis on blocks and run one mission to check command direction and automatic stop.
5. Place only marker 101 and trial scan/alignment/arrival at low speed.
6. Add marker 105 and tune the first right turn, then add 203 and tune the left turn.
7. Add the return markers and tune the U-turn and return turns.
8. Run the full route with a person beside the emergency power switch.

Before every run, place the robot physically at Dock and point it toward marker 101. A software reset cancels state but cannot establish physical pose.
