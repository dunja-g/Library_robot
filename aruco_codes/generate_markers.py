"""
generate_markers.py
Generates 5 printable ArUco marker images (DICT_5X5_50, IDs 0–4).

Run with:  python aruco_codes/generate_markers.py

Each PNG is saved at 600x600 pixels with a white border.
Print each one at 10x10 cm for reliable detection at 0.5–2m range.
"""

import cv2
import os

OUTPUT_DIR = os.path.join(os.path.dirname(__file__))
MARKER_SIZE_PX = 600   # image resolution
BORDER_BITS = 1        # white border around the marker

# Book → ArUco ID mapping (must match pi/book_db.py)
BOOKS = {
    0: "The Great Gatsby",
    1: "1984",
    2: "To Kill a Mockingbird",
    3: "Harry Potter",
    4: "The Hobbit",
}

def main():
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)

    print("Generating ArUco markers...\n")
    for marker_id, book_title in BOOKS.items():
        # Generate the marker image
        marker_img = cv2.aruco.generateImageMarker(
            aruco_dict, marker_id, MARKER_SIZE_PX
        )

        # Add white border padding (makes it easier to detect in real-world photos)
        border_size = 40
        marker_with_border = cv2.copyMakeBorder(
            marker_img,
            border_size, border_size, border_size, border_size,
            cv2.BORDER_CONSTANT,
            value=255  # white
        )

        # Add text label below
        label_height = 80
        import numpy as np
        label = np.full((label_height, marker_with_border.shape[1]), 255, dtype=marker_with_border.dtype)
        cv2.putText(
            label,
            f"ID {marker_id}: {book_title}",
            (10, 52),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.1,
            0,   # black text
            2
        )
        final = np.vstack([marker_with_border, label])

        # Save
        filename = os.path.join(OUTPUT_DIR, f"marker_{marker_id}.png")
        cv2.imwrite(filename, final)
        print(f"  [OK] Saved: marker_{marker_id}.png  ->  {book_title}")

    print(f"\nAll 5 markers saved to: {os.path.abspath(OUTPUT_DIR)}")
    print("\nPrint each image at exactly 10x10 cm.")
    print("Attach one marker to each book, facing toward where the robot will start.")

if __name__ == "__main__":
    main()
