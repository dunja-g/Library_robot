"""Auto-calibration helper tool for fused IMU and encoder robot navigation."""

from __future__ import annotations

import argparse
import sys
from typing import Sequence


def calculate_ticks_per_cm(distance_cm: float, left_ticks: int, right_ticks: int) -> dict:
    """Calculate left, right, and average ticks_per_cm from measured physical distance."""
    if distance_cm <= 0:
        raise ValueError("Measured distance_cm must be positive")
    if left_ticks <= 0 or right_ticks <= 0:
        raise ValueError("Encoder ticks must be positive")

    left_tpcm = left_ticks / distance_cm
    right_tpcm = right_ticks / distance_cm
    avg_tpcm = (left_tpcm + right_tpcm) / 2.0
    asymmetry_pct = abs(left_tpcm - right_tpcm) / avg_tpcm * 100.0

    return {
        "distance_cm": float(distance_cm),
        "left_ticks": int(left_ticks),
        "right_ticks": int(right_ticks),
        "left_ticks_per_cm": round(left_tpcm, 3),
        "right_ticks_per_cm": round(right_tpcm, 3),
        "avg_ticks_per_cm": round(avg_tpcm, 3),
        "asymmetry_percent": round(asymmetry_pct, 2),
        "asymmetry_warning": asymmetry_pct > 5.0,
    }


def format_env_configuration(calibration_results: dict) -> str:
    """Generate suggested .env lines from calibration output."""
    lines = [
        "# --- Fused Navigation Calibration Output ---",
        f"LIBRARY_ROBOT_ENCODER_TICKS_PER_CM={calibration_results['avg_ticks_per_cm']}",
        f"LIBRARY_ROBOT_LEFT_TICKS_PER_CM={calibration_results['left_ticks_per_cm']}",
        f"LIBRARY_ROBOT_RIGHT_TICKS_PER_CM={calibration_results['right_ticks_per_cm']}",
    ]
    if calibration_results.get("asymmetry_warning"):
        lines.append("# WARNING: Left and Right wheel tick difference exceeds 5%. Check wheel alignment/trim!")
    return "\n".join(lines)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Library Robot Fused Odometry Calibration Tool"
    )
    parser.add_argument(
        "--distance",
        type=float,
        default=100.0,
        help="Physical test drive distance in cm (default: 100.0)",
    )
    parser.add_argument(
        "--left-ticks",
        type=int,
        help="Measured left encoder ticks for the test distance",
    )
    parser.add_argument(
        "--right-ticks",
        type=int,
        help="Measured right encoder ticks for the test distance",
    )

    args = parser.parse_args(argv)

    if args.left_ticks is None or args.right_ticks is None:
        print("=== Library Robot Fused Navigation Calibration Guide ===")
        print("1. Ensure IMU is static on flat ground for gyro zero-bias estimation.")
        print("2. Drive robot in a straight line for a measured distance (e.g., 100 cm).")
        print("3. Read raw encoder ticks from serial monitor or telemetry.\n")
        print("Usage Example:")
        print("  python tools/calibrate_fused_navigation.py --distance 100 --left-ticks 820 --right-ticks 835\n")
        return 0

    try:
        results = calculate_ticks_per_cm(args.distance, args.left_ticks, args.right_ticks)
    except ValueError as exc:
        print(f"Calibration error: {exc}", file=sys.stderr)
        return 1

    print("\n--- Calibration Results ---")
    print(f"Distance:           {results['distance_cm']} cm")
    print(f"Left Ticks/cm:      {results['left_ticks_per_cm']}")
    print(f"Right Ticks/cm:     {results['right_ticks_per_cm']}")
    print(f"Average Ticks/cm:   {results['avg_ticks_per_cm']}")
    print(f"Wheel Asymmetry:    {results['asymmetry_percent']}%")

    if results["asymmetry_warning"]:
        print("\n[WARNING] Left/Right encoder asymmetry is > 5%! Check mechanical friction or wheel diameter.")

    print("\n--- Suggested .env snippet ---")
    print(format_env_configuration(results))
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
