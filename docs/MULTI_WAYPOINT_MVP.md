# Archived Multi-Waypoint Prototype

The ArUco waypoint prototype is not loaded by the current application and
must not be used for the borrowing demonstration.

The supported runtime is the marker-free `1A`–`3B` fixed grid:

- QR codes identify students only;
- book destinations come from `pi/book_db.py`;
- encoders measure straight distance and wheel-to-wheel error;
- the MPU6500 provides continuous heading for complementary fusion;
- ultrasonic sensors trigger safety stops;
- pickup confirmation records the loan;
- the space-constrained return route uses supervised reverse movement.

Use `README.md`, `docs/LIVE_DEMO_RUNBOOK.md`,
`docs/FIXED_GRID_ENCODER.md`, `docs/BOOK_NUMBERING.md`, and
`docs/HARDWARE_PINOUT.md` for the current system.

Legacy marker generators, detectors and tests remain only as historical
artifacts.
