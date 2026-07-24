import matplotlib.pyplot as plt
import matplotlib.patches as patches
import os

OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))
IMAGE_PATH = os.path.join(OUTPUT_DIR, "physical_layout_plan.png")

def draw_grid_layout(ax):
    ax.set_title("Mode 2: Fixed Grid Layout (Encoder/IMU)", fontsize=14, fontweight='bold', pad=20)
    ax.set_xlim(-1, 3)
    ax.set_ylim(-1, 4)
    ax.axis('off')

    # Draw Center Aisle
    ax.plot([1, 1], [0, 3.5], linestyle='--', color='gray', label='Center Aisle')

    # Draw Dock
    dock = patches.Rectangle((0.5, -0.8), 1, 0.6, linewidth=2, edgecolor='blue', facecolor='lightblue')
    ax.add_patch(dock)
    ax.text(1, -0.5, "DOCK\n(Facing Row 1)", ha='center', va='center', fontweight='bold')

    # Draw Boxes
    for row in range(1, 4):
        # Column A (Left)
        box_a = patches.Rectangle((-0.8, row - 0.3), 0.6, 0.6, edgecolor='black', facecolor='lightgray')
        ax.add_patch(box_a)
        ax.text(-0.5, row, f"{row}A", ha='center', va='center', fontsize=12)

        # Column B (Right)
        box_b = patches.Rectangle((2.2, row - 0.3), 0.6, 0.6, edgecolor='black', facecolor='lightgray')
        ax.add_patch(box_b)
        ax.text(2.5, row, f"{row}B", ha='center', va='center', fontsize=12)

        # Indicate row lines
        ax.plot([0, 2], [row, row], linestyle=':', color='lightgray')

def draw_aruco_layout(ax):
    ax.set_title("Mode 1: Multi-Waypoint Layout (ArUco)", fontsize=14, fontweight='bold', pad=20)
    ax.set_xlim(-1, 4)
    ax.set_ylim(-1, 5)
    ax.axis('off')

    # Waypoints
    waypoints = {
        0: (0.5, -0.5, "Dock (Marker 0)"),
        101: (0.5, 2, "Main Corridor\n(Marker 101)"),
        105: (2.5, 2, "Zone B Junction\n(Marker 105)"),
        203: (2.5, 4, "Shelf B3\n(Marker 203)")
    }

    for marker_id, (x, y, label) in waypoints.items():
        if marker_id == 0:
            box = patches.Rectangle((x - 0.6, y - 0.3), 1.2, 0.6, edgecolor='blue', facecolor='lightblue')
        else:
            box = patches.Rectangle((x - 0.8, y - 0.4), 1.6, 0.8, edgecolor='green', facecolor='lightgreen')
        ax.add_patch(box)
        ax.text(x, y, label, ha='center', va='center', fontweight='bold')

    # Arrows for path
    ax.annotate('', xy=(0.5, 1.6), xytext=(0.5, 0.1), arrowprops=dict(facecolor='black', shrink=0.05, width=2, headwidth=8))
    ax.annotate('', xy=(1.7, 2), xytext=(1.3, 2), arrowprops=dict(facecolor='black', shrink=0.05, width=2, headwidth=8))
    ax.annotate('', xy=(2.5, 3.6), xytext=(2.5, 2.4), arrowprops=dict(facecolor='black', shrink=0.05, width=2, headwidth=8))

    # Notes
    note = (
        "Outbound Path: Dock -> 101 (Turn Right) -> 105 (Turn Left) -> 203\n"
        "Return Path: 203 (U-Turn) -> 105 (Turn Right) -> 101 (Turn Left) -> Dock\n"
        "Place markers facing the robot's incoming direction."
    )
    ax.text(1.5, -1, note, ha='center', va='center', fontsize=9, bbox=dict(facecolor='yellow', alpha=0.3))


fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 8))

draw_grid_layout(ax1)
draw_aruco_layout(ax2)

plt.tight_layout()
plt.savefig(IMAGE_PATH, dpi=300, bbox_inches='tight')
print(f"Layout diagram saved to {IMAGE_PATH}")
