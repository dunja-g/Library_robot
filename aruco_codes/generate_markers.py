"""Generate printable ArUco markers for the library books."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np


DEFAULT_DICTIONARY = "DICT_5X5_50"
DEFAULT_MARKER_IDS = tuple(range(5))
BOOK_TITLES = {
    0: "The Great Gatsby",
    1: "1984",
    2: "To Kill a Mockingbird",
    3: "Harry Potter",
    4: "The Hobbit",
}


def get_dictionary(name: str = DEFAULT_DICTIONARY):
    """Return an OpenCV ArUco dictionary by its public constant name."""
    if not hasattr(cv2, "aruco"):
        raise RuntimeError(
            "OpenCV ArUco is unavailable. Install opencv-contrib-python."
        )
    dictionary_id = getattr(cv2.aruco, name, None)
    if dictionary_id is None:
        raise ValueError(f"Unknown ArUco dictionary: {name}")
    return cv2.aruco.getPredefinedDictionary(dictionary_id)


def create_marker(
    marker_id: int,
    image_size: int = 300,
    border_px: int = 20,
    dictionary_name: str = DEFAULT_DICTIONARY,
) -> np.ndarray:
    """Create one square uint8 marker image with a printable white border."""
    if marker_id < 0:
        raise ValueError("marker_id must be non-negative")
    if image_size <= 0:
        raise ValueError("image_size must be positive")
    if border_px < 0 or border_px * 2 >= image_size:
        raise ValueError("border_px must leave a positive marker area")

    dictionary = get_dictionary(dictionary_name)
    marker_size = image_size - 2 * border_px

    if hasattr(cv2.aruco, "generateImageMarker"):
        marker = cv2.aruco.generateImageMarker(
            dictionary, marker_id, marker_size
        )
    else:  # OpenCV contrib < 4.7
        marker = np.zeros((marker_size, marker_size), dtype=np.uint8)
        cv2.aruco.drawMarker(dictionary, marker_id, marker_size, marker, 1)

    image = np.full((image_size, image_size), 255, dtype=np.uint8)
    end = border_px + marker_size
    image[border_px:end, border_px:end] = marker
    return image


def save_markers(
    output_dir: str | Path,
    marker_ids: Iterable[int] = DEFAULT_MARKER_IDS,
    image_size: int = 300,
    border_px: int = 20,
    dictionary_name: str = DEFAULT_DICTIONARY,
    include_labels: bool = False,
) -> list[Path]:
    """Generate markers and return the paths written to disk."""
    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)

    written: list[Path] = []
    for marker_id in marker_ids:
        image = create_marker(
            marker_id,
            image_size=image_size,
            border_px=border_px,
            dictionary_name=dictionary_name,
        )
        if include_labels:
            label_height = max(60, image_size // 8)
            label = np.full(
                (label_height, image_size), 255, dtype=image.dtype
            )
            title = BOOK_TITLES.get(marker_id, "Unknown book")
            font_scale = max(0.55, image_size / 550.0)
            cv2.putText(
                label,
                f"ID {marker_id}: {title}",
                (10, int(label_height * 0.68)),
                cv2.FONT_HERSHEY_SIMPLEX,
                font_scale,
                0,
                2,
                cv2.LINE_AA,
            )
            image = np.vstack([image, label])
        output_path = destination / f"marker_{marker_id}.png"
        if not cv2.imwrite(str(output_path), image):
            raise OSError(f"Failed to write marker image: {output_path}")
        written.append(output_path)
        print(f"Saved ArUco marker {marker_id}: {output_path}")
    return written


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parent,
        help="Directory for marker_<ID>.png files",
    )
    parser.add_argument(
        "--ids",
        type=int,
        nargs="+",
        default=list(DEFAULT_MARKER_IDS),
        help="Marker IDs to generate (default: 0 1 2 3 4)",
    )
    parser.add_argument("--image-size", type=int, default=600)
    parser.add_argument("--border-px", type=int, default=40)
    parser.add_argument("--dictionary", default=DEFAULT_DICTIONARY)
    parser.add_argument(
        "--without-labels",
        action="store_true",
        help="Do not add the book title below each printable marker",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    save_markers(
        args.output_dir,
        marker_ids=args.ids,
        image_size=args.image_size,
        border_px=args.border_px,
        dictionary_name=args.dictionary,
        include_labels=not args.without_labels,
    )


if __name__ == "__main__":
    main()

