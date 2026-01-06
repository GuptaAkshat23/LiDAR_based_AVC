import open3d as o3d
import numpy as np
from scipy.spatial import cKDTree
import tkinter as tk
from tkinter import filedialog, messagebox
import os
import sys

# ================= CONFIGURATION =================
# Points within this distance (meters) of the Zero Plane are considered "Background"
# Increase this if parts of the walls are appearing in the Object file.
# Decrease this if the Object is disappearing into the Background file.
DIST_THRESHOLD = 0.05  # 5cm


# =================================================

def load_pcd(filepath):
    print(f"📖 Loading: {os.path.basename(filepath)}...")
    pcd = o3d.io.read_point_cloud(filepath)
    if pcd.is_empty():
        print("❌ Error: File is empty.")
        return None
    return pcd


def main():
    root = tk.Tk()
    root.withdraw()

    print("--- DUAL EXTRACTION TOOL ---")

    # 1. Select Files
    print("1. Select ZERO PLANE (Background)...")
    zero_path = filedialog.askopenfilename(title="Select ZERO PLANE (Background) PCD",
                                           filetypes=[("PCD Files", "*.pcd")])
    if not zero_path: return

    print("2. Select STACKED DATA (Scan)...")
    stack_path = filedialog.askopenfilename(title="Select STACKED (Vehicle) PCD", filetypes=[("PCD Files", "*.pcd")])
    if not stack_path: return

    output_dir = os.path.dirname(stack_path)

    # 2. Load Data
    pcd_bg = load_pcd(zero_path)
    pcd_stack = load_pcd(stack_path)

    pts_bg = np.asarray(pcd_bg.points)
    pts_stack = np.asarray(pcd_stack.points)

    # 3. Coordinate Mapping (Crucial Step)
    # Zero Plane: Uses X, Y for the ring shape.
    # Stacked Data: Uses Y, Z for the ring shape (X is travel distance).
    bg_2d = pts_bg[:, 0:2]
    stack_cross_section = pts_stack[:, 1:3]

    print(f"\n⚙️  Processing {len(pts_stack)} points...")

    # 4. Build KDTree for Background
    tree = cKDTree(bg_2d)

    # 5. Calculate Distances
    # Find distance from every Stack point to the nearest Background point
    distances, _ = tree.query(stack_cross_section, k=1)

    # 6. Split Data
    # Indices of points that are FAR from background (The Object)
    foreground_indices = np.where(distances > DIST_THRESHOLD)[0]

    # Indices of points that are CLOSE to background (The Removed Part)
    background_indices = np.where(distances <= DIST_THRESHOLD)[0]

    # 7. Save Files
    print(f"\n📊 Results:")

    # --- SAVE FOREGROUND (Object) ---
    if len(foreground_indices) > 0:
        pcd_fg = o3d.geometry.PointCloud()
        pcd_fg.points = o3d.utility.Vector3dVector(pts_stack[foreground_indices])
        fg_path = os.path.join(output_dir, "EXTRACTED_FOREGROUND.pcd")
        o3d.io.write_point_cloud(fg_path, pcd_fg)
        print(f"   ✅ Saved Object ({len(foreground_indices)} pts) -> {os.path.basename(fg_path)}")
    else:
        print("   ⚠️ No Foreground detected (Threshold too high?)")

    # --- SAVE BACKGROUND (The Removed Part) ---
    if len(background_indices) > 0:
        pcd_bg_out = o3d.geometry.PointCloud()
        pcd_bg_out.points = o3d.utility.Vector3dVector(pts_stack[background_indices])
        bg_path = os.path.join(output_dir, "REMOVED_BACKGROUND.pcd")
        o3d.io.write_point_cloud(bg_path, pcd_bg_out)
        print(f"   ✅ Saved Background ({len(background_indices)} pts) -> {os.path.basename(bg_path)}")

        # Determine likely intent
        if len(foreground_indices) == 0:
            messagebox.showinfo("Result",
                                f"All points were classified as Background.\nCheck 'REMOVED_BACKGROUND.pcd' to see your data.")
        else:
            messagebox.showinfo("Success",
                                f"Split Complete!\n\nObject: {len(foreground_indices)} pts\nBackground: {len(background_indices)} pts")
    else:
        print("   ⚠️ No Background detected (Threshold too low or Misalignment)")
        messagebox.showwarning("Warning", "No background points found. Check alignment!")


if __name__ == "__main__":
    main()