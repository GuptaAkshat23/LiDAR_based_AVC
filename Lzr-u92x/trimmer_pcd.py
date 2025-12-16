import open3d as o3d
import numpy as np
import os
import tkinter as tk
from tkinter import filedialog

# --- CONFIGURATION ---
X_MIN = 0.0  # Start
X_MAX = 0.33  # End

Y_MIN = -0.35
Y_MAX = 0.35

OUTPUT_FOLDER_NAME = "trimmed_down_pcd"


# ---------------------

def pick_files():
    """Opens a dialog to pick MULTIPLE files"""
    root = tk.Tk()
    root.withdraw()
    # askopenfilenames (plural) returns a tuple of file paths
    file_paths = filedialog.askopenfilenames(
        title="Select PCD Files to Trim (Select All with Ctrl+A)",
        filetypes=[("Point Cloud Data", "*.pcd")]
    )
    return root.tk.splitlist(file_paths)


def trim_files():
    # 1. User Selects Files directly
    files = pick_files()

    if not files:
        print("❌ No files selected.")
        return

    print(f"📄 Selected {len(files)} files for processing...")

    # 2. Create Output Folder (based on the first file's location)
    first_file_dir = os.path.dirname(files[0])
    output_folder = os.path.join(first_file_dir, OUTPUT_FOLDER_NAME)

    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
        print(f"📂 Created output folder: {output_folder}")

    print(f"🔪 Trimming Limits: X[{X_MIN} to {X_MAX}] | Y[{Y_MIN} to {Y_MAX}]")

    # 3. Process
    for file_path in files:
        filename = os.path.basename(file_path)

        try:
            # Read
            pcd = o3d.io.read_point_cloud(file_path)
            if pcd.is_empty():
                print(f"⚠️ {filename} is empty/corrupt.")
                continue

            points = np.asarray(pcd.points)

            # Filter Logic
            mask = (
                    (points[:, 0] >= X_MIN) & (points[:, 0] <= X_MAX) &
                    (points[:, 1] >= Y_MIN) & (points[:, 1] <= Y_MAX)
            )

            trimmed_pcd = pcd.select_by_index(np.where(mask)[0])

            # Save
            if len(trimmed_pcd.points) > 0:
                output_path = os.path.join(output_folder, filename)
                o3d.io.write_point_cloud(output_path, trimmed_pcd)
                # print(f"   Saved: {filename}")
            else:
                print(f"⚠️ {filename}: Result empty (All points outside range).")

        except Exception as e:
            print(f"❌ Error processing {filename}: {e}")

    print("\n✅ Processing Complete.")
    print(f"📂 Trimmed files saved in: {output_folder}")


if __name__ == "__main__":
    trim_files()