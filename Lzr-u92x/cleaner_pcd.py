import open3d as o3d
import numpy as np
import os
import tkinter as tk
from tkinter import filedialog

# --- NOISE REMOVAL SETTINGS ---
NB_NEIGHBORS = 20
STD_RATIO = 2.0
MIN_DIST = 0.1
MAX_DIST = 60.0


# ------------------------------

def pick_file():
    """Opens a standard Windows file picker"""
    root = tk.Tk()
    root.withdraw()
    file_path = filedialog.askopenfilename(
        title="Select a LiDAR Scan to Clean (.pcd)",
        filetypes=[("Point Cloud Data", "*.pcd")]
    )
    return file_path


def clean_point_cloud():
    # 1. Select File
    input_path = pick_file()
    if not input_path:
        print("❌ No file selected.")
        return

    print(f"📂 Loading: {os.path.basename(input_path)}...")
    pcd = o3d.io.read_point_cloud(input_path)

    if pcd.is_empty():
        print("⚠️ File is empty.")
        return

    print(f"   Original Points: {len(pcd.points)}")

    # --- STEP 1: REMOVE ZERO/INVALID POINTS ---
    points = np.asarray(pcd.points)
    distances = np.linalg.norm(points, axis=1)

    valid_ind = np.where((distances > MIN_DIST) & (distances < MAX_DIST))[0]
    pcd_filtered = pcd.select_by_index(valid_ind)

    removed_range = len(pcd.points) - len(pcd_filtered.points)
    print(f"   Step 1 (Range): Removed {removed_range} invalid points.")

    # --- STEP 2: REMOVE GHOST POINTS ---
    print(f"   Step 2 (Statistical): Running SOR filter...")

    clean_pcd, ind = pcd_filtered.remove_statistical_outlier(
        nb_neighbors=NB_NEIGHBORS,
        std_ratio=STD_RATIO
    )

    removed_stat = len(pcd_filtered.points) - len(clean_pcd.points)
    print(f"   Step 2 (Statistical): Removed {removed_stat} noise points.")
    print(f"✅ Final Point Count: {len(clean_pcd.points)}")

    # --- STEP 3: VISUALIZATION ---
    print("\n👁️ Opening Visualizer (Close window to Save)...")
    print("   RED  = Noise (Deleted)")
    print("   GREY = Clean Data (Kept)")

    noise_pcd = pcd_filtered.select_by_index(ind, invert=True)
    noise_pcd.paint_uniform_color([1, 0, 0])  # Red
    clean_pcd.paint_uniform_color([0.5, 0.5, 0.5])  # Grey

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Noise Removal Result", width=1024, height=768)
    opt = vis.get_render_option()
    opt.background_color = np.asarray([0, 0, 0])
    opt.point_size = 2.0
    vis.add_geometry(clean_pcd)
    vis.add_geometry(noise_pcd)
    vis.run()
    vis.destroy_window()

    # --- STEP 4: SAVE TO 'cleaned_scans' FOLDER ---

    # 1. Get the directory of the ORIGINAL file
    original_folder = os.path.dirname(input_path)

    # 2. Create the new sub-folder path
    output_folder = os.path.join(original_folder, "cleaned_scans")

    # 3. Create it if it doesn't exist
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
        print(f"📂 Created new folder: {output_folder}")

    # 4. Generate filename
    filename = os.path.basename(input_path)
    name, ext = os.path.splitext(filename)
    output_filename = f"{name}_CLEANED{ext}"
    output_path = os.path.join(output_folder, output_filename)

    # 5. Write file
    o3d.io.write_point_cloud(output_path, clean_pcd)
    print(f"\n💾 Saved successfully to:\n   {output_path}")


if __name__ == "__main__":
    clean_point_cloud()