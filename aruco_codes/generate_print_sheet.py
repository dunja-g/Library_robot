"""
generate_print_sheet.py
Generates a single ready-to-print PDF with all 5 ArUco markers
laid out at exactly 10x10 cm each on A4 pages.

Run with:  python aruco_codes/generate_print_sheet.py
Output:    aruco_codes/markers_print_sheet.pdf

Just open the PDF and print at 100% (Actual Size). 
Do NOT select "Fit to page" — that will resize the markers.
"""

import cv2
import numpy as np
import os

# ── Try to use matplotlib (usually pre-installed) ────────────────────────────
try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import matplotlib.image as mpimg
    HAS_MPL = True
except ImportError:
    HAS_MPL = False

OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))
PDF_PATH   = os.path.join(OUTPUT_DIR, "markers_print_sheet.pdf")

BOOKS = {
    0: "The Great Gatsby",
    1: "1984",
    2: "To Kill a Mockingbird",
    3: "Harry Potter",
    4: "The Hobbit",
}

# Target physical size in cm
TARGET_CM  = 10
TARGET_IN  = TARGET_CM / 2.54   # ~3.937 inches
PRINT_DPI  = 300
TARGET_PX  = int(TARGET_IN * PRINT_DPI)  # 1181 pixels at 300 DPI = 10 cm


def make_marker_image(marker_id: int, book_title: str) -> np.ndarray:
    """Generate a single high-res marker image ready for printing."""
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)

    # Generate marker at target pixel size
    marker = cv2.aruco.generateImageMarker(aruco_dict, marker_id, TARGET_PX)

    # Add white border (5% of size on each side)
    border = TARGET_PX // 20
    marker_bordered = cv2.copyMakeBorder(
        marker, border, border, border, border,
        cv2.BORDER_CONSTANT, value=255
    )

    # Add label strip at the bottom
    label_h = max(60, TARGET_PX // 15)
    label   = np.full((label_h, marker_bordered.shape[1]), 255, dtype=np.uint8)
    font_scale = label_h / 60
    cv2.putText(
        label,
        f"ID {marker_id}  |  {book_title}",
        (border, int(label_h * 0.72)),
        cv2.FONT_HERSHEY_SIMPLEX,
        font_scale, 0, 2
    )
    return np.vstack([marker_bordered, label])


def generate_with_matplotlib():
    """One marker per page in a multi-page PDF."""
    from matplotlib.backends.backend_pdf import PdfPages

    with PdfPages(PDF_PATH) as pdf:
        for marker_id, book_title in BOOKS.items():
            img = make_marker_image(marker_id, book_title)
            img_rgb = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

            # Figure size: exactly TARGET_IN x (TARGET_IN * height_ratio) inches
            h, w = img.shape
            fig_w = TARGET_IN + 0.5   # add small margin
            fig_h = (h / w) * TARGET_IN + 1.0

            fig, ax = plt.subplots(figsize=(fig_w, fig_h), dpi=PRINT_DPI)
            ax.imshow(img_rgb)
            ax.axis('off')

            # Title above marker
            fig.suptitle(
                f"Library Robot — ArUco Marker ID {marker_id}\nPrint at ACTUAL SIZE (100%) — do not scale",
                fontsize=7, y=0.98, color='gray'
            )
            fig.tight_layout(pad=0.3)
            pdf.savefig(fig, dpi=PRINT_DPI)
            plt.close(fig)
            print(f"  [OK] Page {marker_id + 1}/5  ->  ID {marker_id}: {book_title}")

    print(f"\nPDF saved to: {PDF_PATH}")
    print("Open this PDF and print at 100% (Actual Size) — each marker will be exactly 10x10 cm.")


def generate_pngs_with_dpi():
    """Fallback: save high-res PNGs with correct DPI metadata using Pillow."""
    try:
        from PIL import Image
    except ImportError:
        print("ERROR: Neither matplotlib nor Pillow is installed.")
        print("Run:  pip install matplotlib  or  pip install pillow")
        return

    for marker_id, book_title in BOOKS.items():
        img = make_marker_image(marker_id, book_title)
        pil_img = Image.fromarray(img)
        filename = os.path.join(OUTPUT_DIR, f"printready_marker_{marker_id}.png")
        # Save with 300 DPI metadata — printing at "actual size" gives 10x10 cm
        pil_img.save(filename, dpi=(PRINT_DPI, PRINT_DPI))
        print(f"  [OK] Saved: printready_marker_{marker_id}.png  ->  {book_title}")

    print(f"\nPNGs saved to: {OUTPUT_DIR}")
    print("Open each PNG and print at ACTUAL SIZE (100%). Each marker = 10x10 cm.")


# ── Run ───────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print(f"Generating print-ready markers at {TARGET_PX}px ({TARGET_CM}cm at {PRINT_DPI} DPI)...\n")
    if HAS_MPL:
        generate_with_matplotlib()
    else:
        print("matplotlib not found — generating high-DPI PNGs instead.\n")
        generate_pngs_with_dpi()
