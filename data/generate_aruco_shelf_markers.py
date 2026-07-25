"""Generate labelled ArUco markers for the library shelf grid.

Layout (7 markers total):
  - Center aisle between the two bookshelf columns
  - Six shelf boxes: 1A, 1B, 2A, 2B, 3A, 3B

Uses DICT_5X5_50 (same as aruco_codes/generate_markers.py).
Output: 480x480 PNG with a 400x400 marker and a labelled footer strip.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

DEFAULT_DICTIONARY = "DICT_5X5_50"
IMAGE_SIZE = 480
MARKER_SIZE = 400
MARKER_ORIGIN = 40
LABEL_HEIGHT = 40

# ID -> (filename slug, short map label, placement description)
# ID 0 and ID 6 are swapped on the physical map (no reprint): the printed
# ID 0 marker sits at center aisle; the printed ID 6 marker sits at box 1A.
MARKER_CATALOG = {
    0: ("center_aisle", "CENTER", "Center aisle between bookshelf rows"),
    1: ("shelf_1B", "1B", "Row 1 right box"),
    2: ("shelf_2A", "2A", "Row 2 left box"),
    3: ("shelf_2B", "2B", "Row 2 right box"),
    4: ("shelf_3A", "3A", "Row 3 left box"),
    5: ("shelf_3B", "3B", "Row 3 right box"),
    6: ("shelf_1A", "1A", "Row 1 left box"),
}


def get_dictionary(name: str = DEFAULT_DICTIONARY):
    dictionary_id = getattr(cv2.aruco, name, None)
    if dictionary_id is None:
        raise ValueError(f"Unknown ArUco dictionary: {name}")
    return cv2.aruco.getPredefinedDictionary(dictionary_id)


def _centered_text_x(text: str, font_scale: float, thickness: int) -> int:
    (text_width, _), _ = cv2.getTextSize(
        text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness
    )
    return max(8, (IMAGE_SIZE - text_width) // 2)


def create_marker_image(
    marker_id: int,
    *,
    dictionary_name: str = DEFAULT_DICTIONARY,
) -> np.ndarray:
    if marker_id not in MARKER_CATALOG:
        raise ValueError(f"Unsupported marker ID: {marker_id}")

    slug, map_label, _placement = MARKER_CATALOG[marker_id]
    dictionary = get_dictionary(dictionary_name)

    if hasattr(cv2.aruco, "generateImageMarker"):
        marker = cv2.aruco.generateImageMarker(
            dictionary, marker_id, MARKER_SIZE
        )
    else:
        marker = np.zeros((MARKER_SIZE, MARKER_SIZE), dtype=np.uint8)
        cv2.aruco.drawMarker(dictionary, marker_id, MARKER_SIZE, marker, 1)

    canvas = np.full((IMAGE_SIZE, IMAGE_SIZE), 255, dtype=np.uint8)
    end = MARKER_ORIGIN + MARKER_SIZE
    canvas[MARKER_ORIGIN:end, MARKER_ORIGIN:end] = marker

    label = np.full((LABEL_HEIGHT, IMAGE_SIZE), 255, dtype=np.uint8)
    footer = f"ID {marker_id}  |  {map_label}"
    font_scale = 0.72
    thickness = 2
    text_x = _centered_text_x(footer, font_scale, thickness)
    cv2.putText(
        label,
        footer,
        (text_x, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        font_scale,
        0,
        thickness,
        cv2.LINE_AA,
    )
    canvas[IMAGE_SIZE - LABEL_HEIGHT :] = label
    return canvas


def marker_output_path(output_dir: Path, marker_id: int) -> Path:
    slug, _, _ = MARKER_CATALOG[marker_id]
    return output_dir / f"aruco_marker_{marker_id}_{slug}.png"


def save_markers(
    output_dir: str | Path,
    marker_ids: list[int] | None = None,
    *,
    dictionary_name: str = DEFAULT_DICTIONARY,
) -> list[Path]:
    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    ids = sorted(MARKER_CATALOG) if marker_ids is None else marker_ids

    written: list[Path] = []
    for marker_id in ids:
        image = create_marker_image(marker_id, dictionary_name=dictionary_name)
        output_path = marker_output_path(destination, marker_id)
        if not cv2.imwrite(str(output_path), image):
            raise OSError(f"Failed to write marker image: {output_path}")
        written.append(output_path)
        _, map_label, placement = MARKER_CATALOG[marker_id]
        print(f"Saved ID {marker_id} ({map_label}) -> {output_path.name}  [{placement}]")
    return written


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parent,
        help="Directory for aruco_marker_<ID>_*.png files",
    )
    parser.add_argument(
        "--ids",
        type=int,
        nargs="+",
        default=sorted(MARKER_CATALOG),
        help="Marker IDs to generate (default: all 7)",
    )
    parser.add_argument("--dictionary", default=DEFAULT_DICTIONARY)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    save_markers(args.output_dir, marker_ids=args.ids, dictionary_name=args.dictionary)


if __name__ == "__main__":
    main()
