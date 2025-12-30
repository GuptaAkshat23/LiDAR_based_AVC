import open3d as o3d
import numpy as np
import cv2
import os
import tkinter as tk
from tkinter import filedialog

# ==============================
# CONFIGURATION (UPDATED FOR LARGER VIEW)
# ==============================
# Increase ranges to capture the whole scene
# Decrease GRID_RES to increase pixel count (zoom in)
X_RANGE = (-0, 2)  # Expanded to 40 meters wide
Y_RANGE = (-0.5, 2)  # Expanded to 60 meters long
GRID_RES = 0.005  # 2cm per pixel (Higher resolution = Larger image)
MAX_DIST = 50.0  # Adjusted to match the longer Y_RANGE


# ==============================
# FILE PICKER
# ==============================
def select_pcd_file():
    root = tk.Tk()
    root.withdraw()
    file_path = filedialog.askopenfilename(
        title="Select PCD File",
        filetypes=[("Point Cloud Data", "*.pcd")]
    )
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

    # Output image path
    output_image = os.path.splitext(pcd_path)[0] + "_BEV.png"

    # ==============================
    # LOAD PCD
    # ==============================
    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points)

    if len(points) == 0:
        print("❌ Empty PCD file")
        return

    # ==============================
    # ROI FILTER
    # ==============================
    mask = (
            (points[:, 0] >= X_RANGE[0]) & (points[:, 0] <= X_RANGE[1]) &
            (points[:, 1] >= Y_RANGE[0]) & (points[:, 1] <= Y_RANGE[1])
    )
    points = points[mask]

    # ==============================
    # CREATE BEV GRID
    # ==============================
    x_bins = int((X_RANGE[1] - X_RANGE[0]) / GRID_RES)
    y_bins = int((Y_RANGE[1] - Y_RANGE[0]) / GRID_RES)

    bev = np.zeros((y_bins, x_bins), dtype=np.float32)

    # ==============================
    # PROJECT POINTS
    # ==============================
    # Optimization: Use vectorized numpy instead of a loop for speed
    for x, y, z in points:
        xi = int((x - X_RANGE[0]) / GRID_RES)
        yi = int((y - Y_RANGE[0]) / GRID_RES)

        if 0 <= xi < x_bins and 0 <= yi < y_bins:
            dist = np.sqrt(x ** 2 + y ** 2)
            intensity = 1.0 - min(dist / MAX_DIST, 1.0)
            # Take the maximum intensity for that pixel
            if intensity > bev[yi, xi]:
                bev[yi, xi] = intensity

    # ==============================
    # IMAGE PROCESSING
    # ==============================
    bev = (bev * 255).astype(np.uint8)
    bev = np.flipud(bev)  # front at top

    # Optional: Slightly larger blur for the higher resolution
    bev = cv2.GaussianBlur(bev, (3, 3), 0)

    # ==============================
    # SAVE IMAGE
    # ==============================
    cv2.imwrite(output_image, bev)
    print(f"✅ BEV image saved: {output_image}")
    print(f"📏 Final Image Size: {x_bins}x{y_bins} pixels")


# ==============================
# RUN
# ==============================
if __name__ == "__main__":
    main()