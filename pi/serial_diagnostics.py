"""Safely test Arduino Mega serial and ultrasonic communication."""

from __future__ import annotations

import argparse
import logging
import time

try:
    from .serial_bridge import SerialBridge
except ImportError:  # Supports ``python pi/serial_diagnostics.py``.
    from serial_bridge import SerialBridge


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--samples", type=int, default=5)
    parser.add_argument(
        "--motor-test",
        action="store_true",
        help="Explicitly enable short wheel movement tests; raise wheels first",
    )
    parser.add_argument("--movement-seconds", type=float, default=0.5)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.samples <= 0:
        raise ValueError("samples must be positive")
    if not 0.1 <= args.movement_seconds <= 2.0:
        raise ValueError("movement-seconds must be between 0.1 and 2.0")

    logging.basicConfig(level=logging.INFO)
    bridge = SerialBridge(port=args.port)
    try:
        for index in range(args.samples):
            print(f"Ultrasonic {index + 1}/{args.samples}: {bridge.get_ultrasonic()}")
            time.sleep(0.2)

        if args.motor_test:
            print("Motor test enabled. Wheels must be raised off the ground.")
            for name, command in (
                ("FORWARD", bridge.send_forward),
                ("ROTATE_LEFT", bridge.send_rotate_left),
                ("ROTATE_RIGHT", bridge.send_rotate_right),
            ):
                print(name)
                command()
                time.sleep(args.movement_seconds)
                bridge.send_stop()
                time.sleep(0.3)
    finally:
        bridge.close()


if __name__ == "__main__":
    main()
