import open3d as o3d
import numpy as np
import cv2
import os
import tkinter as tk
from tkinter import filedialog

# ==============================
# CONFIGURATION
# ==============================
GRID_RES = 0.01  # meters per pixel
EDGE_MARGIN = 0.10  # meters

# --- ROTATION SETTING (X-AXIS) ---
# Angle in degrees to rotate around the X-axis (Pitch).
# Try 5.0, -5.0, 10.0 to correct the bottom-up perspective.
ROTATION_ANGLE_X = 40.0

# --- CONTRAST & COLOR SETTINGS ---
CONTRAST_GAMMA = 1.0  # 1.0 = Natural, 2.0 = High Contrast
MIN_BRIGHTNESS = 0.3  # 0.3 = Dark Grey points (instead of black)

# Range Clipping
BLACK_POINT_PCT = 2
WHITE_POINT_PCT = 98

# Background (0 = Black)
BACKGROUND_COLOR = 0


# ==============================
# FILE PICKER
# ==============================
def select_pcd_file():
    root = tk.Tk()
    root.withdraw()
    root.lift()
    root.attributes('-topmost', True)
    file_path = filedialog.askopenfilename(
        title="Select PCD File",
        filetypes=[("Point Cloud Data", "*.pcd")]
    )
    root.destroy()
    return file_path


# ==============================
# MAIN
# ==============================
def main():
    pcd_path = select_pcd_file()
    if not pcd_path:
        print("❌ No file selected")
        return

    print(f"📂 Selected: {os.path.basename(pcd_path)}")
    output_image = os.path.splitext(pcd_path)[0] + f"_RotX_{ROTATION_ANGLE_X}.png"

    # ==============================
    # LOAD PCD
    # ==============================
    pcd = o3d.io.read_point_cloud(pcd_path)

    if len(pcd.points) == 0:
        print("❌ Empty PCD file")
        return

    # ==============================
    # APPLY ROTATION (X-AXIS)
    # ==============================
    if ROTATION_ANGLE_X != 0:
        print(f"🔄 Rotating by {ROTATION_ANGLE_X} degrees around X-axis...")

        # Convert degrees to radians
        angle_rad = np.radians(ROTATION_ANGLE_X)

        # Create Rotation Matrix for X-axis
        # [ 1    0        0   ]
        # [ 0  cos(a)  -sin(a)]
        # [ 0  sin(a)   cos(a)]
        # Note: (angle_rad, 0, 0) specifies X, Y, Z angles respectively
        R = pcd.get_rotation_matrix_from_xyz((angle_rad, 0, 0))

        # Apply rotation (center=(0,0,0) rotates around the origin)
        pcd.rotate(R, center=(0, 0, 0))

    # Convert to NumPy for processing
    points = np.asarray(pcd.points)

    # ==============================
    # DEPTH CALCULATION
    # ==============================
    depth_values = points[:, 2]  # Assuming Z is depth

    min_depth = np.percentile(depth_values, BLACK_POINT_PCT)
    max_depth = np.percentile(depth_values, WHITE_POINT_PCT)
    depth_range = max_depth - min_depth
    if depth_range < 1e-6: depth_range = 1.0

    print(f"📏 Depth Range: {min_depth:.2f} to {max_depth:.2f}")

    # ==============================
    # FRAME SETUP
    # ==============================
    min_x, max_x = points[:, 0].min(), points[:, 0].max()
    min_y, max_y = points[:, 1].min(), points[:, 1].max()

    min_x -= EDGE_MARGIN;
    max_x += EDGE_MARGIN
    min_y -= EDGE_MARGIN;
    max_y += EDGE_MARGIN

    x_bins = int(np.ceil((max_x - min_x) / GRID_RES))
    y_bins = int(np.ceil((max_y - min_y) / GRID_RES))

    # Initialize grid
    bev = np.full((y_bins, x_bins), -1.0, dtype=np.float32)

    # ==============================
    # PROJECT POINTS
    # ==============================
    print("🔨 Processing...")

    xi_arr = ((points[:, 0] - min_x) / GRID_RES).astype(np.int32)
    yi_arr = ((points[:, 1] - min_y) / GRID_RES).astype(np.int32)
    valid_mask = (xi_arr >= 0) & (xi_arr < x_bins) & (yi_arr >= 0) & (yi_arr < y_bins)

    # Normalize & Brightness Logic
    norm_depths = (depth_values - min_depth) / depth_range
    norm_depths = np.clip(norm_depths, 0.0, 1.0)
    norm_depths = np.power(norm_depths, CONTRAST_GAMMA)

    # Apply Grey Scale lift
    norm_depths = MIN_BRIGHTNESS + (norm_depths * (1.0 - MIN_BRIGHTNESS))

    for i in np.where(valid_mask)[0]:
        xi, yi = xi_arr[i], yi_arr[i]
        val = norm_depths[i]
        if val > bev[yi, xi]:
            bev[yi, xi] = val

    # ==============================
    # IMAGE GENERATION
    # ==============================
    bg_norm = BACKGROUND_COLOR / 255.0
    bev[bev == -1] = bg_norm

    bev_uint8 = (bev * 255).astype(np.uint8)
    bev_final = np.flipud(bev_uint8)

    cv2.imwrite(output_image, bev_final)
    print(f"✅ Saved: {output_image}")


if __name__ == "__main__":
    main()