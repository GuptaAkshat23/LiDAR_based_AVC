import sys
import os
import struct
import numpy as np
import tkinter as tk
from tkinter import filedialog, messagebox

# =====================================================
# ALGORITHM CONFIGURATION (Wu et al. 2018)
# =====================================================
# "The recommended side length of the cube is 0.1 m" [cite: 127]
GRID_CELL_SIZE = 0.10


# =====================================================
# FILE I/O HELPERS
# =====================================================
def read_pcd(filename):
    """
    Reads an ASCII PCD file and returns a list of (x, y, z) tuples.
    """
    points = []
    if not filename:
        return []

    print(f"📖 Reading: {os.path.basename(filename)}...")
    try:
        with open(filename, 'r') as f:
            lines = f.readlines()
            data_section = False

            for line in lines:
                if line.startswith("DATA ascii"):
                    data_section = True
                    continue

                if data_section:
                    parts = line.strip().split()
                    if len(parts) >= 3:
                        try:
                            # Parse X, Y, Z
                            x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                            points.append((x, y, z))
                        except ValueError:
                            continue
    except Exception as e:
        messagebox.showerror("Error", f"Failed to read file:\n{e}")
        return []

    return points


def save_pcd(points, filename):
    """
    Saves a list of points to a standard PCD format.
    """
    if not points:
        messagebox.showwarning("Warning", "No object points remained after filtering!")
        return

    print(f"💾 Saving to: {os.path.basename(filename)}...")
    header = (
        "# .PCD v0.7 - GUI FILTERED EXTRACTION\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {len(points)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\n"
        "DATA ascii\n"
    )

    try:
        with open(filename, "w") as f:
            f.write(header)
            for p in points:
                f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")

        print("✅ Success.")
    except Exception as e:
        messagebox.showerror("Error", f"Failed to save file:\n{e}")


# =====================================================
# CORE ALGORITHM: BACKGROUND MATRIX
# =====================================================
def perform_filtering(zero_plane_file, stacked_file, output_file):
    # 1. Read Data
    bg_points = read_pcd(zero_plane_file)
    stacked_points = read_pcd(stacked_file)

    if not bg_points:
        print("❌ Error: Zero Plane file is empty or invalid.")
        return
    if not stacked_points:
        print("❌ Error: Stacked Scan file is empty or invalid.")
        return

    # 2. Build Background Matrix (Learning Phase)
    # The paper uses a spatial grid to identify background areas[cite: 84, 101].
    print(f"⚙️  Building Background Matrix (Cell Size: {GRID_CELL_SIZE}m)...")
    background_matrix = set()

    for p in bg_points:
        x, y = p[0], p[1]
        # Calculate grid indices (ignore Z for 2D background definition)
        ix = int(np.floor(x / GRID_CELL_SIZE))
        iy = int(np.floor(y / GRID_CELL_SIZE))
        background_matrix.add((ix, iy))

    print(f"   Mapped {len(background_matrix)} background grid cells.")

    # 3. Filter Stacked Data (Real-time Phase)
    # "Can the location of points be found in the background matrix?" [cite: 102]
    print("🔍 Filtering objects...")
    final_points = []

    for p in stacked_points:
        x, y, z = p[0], p[1], p[2]

        ix = int(np.floor(x / GRID_CELL_SIZE))
        iy = int(np.floor(y / GRID_CELL_SIZE))

        # If the X,Y coordinate is in the matrix, it's background -> Skip it.
        # If NOT in matrix, it's a moving object -> Keep it.
        if (ix, iy) not in background_matrix:
            final_points.append(p)

    # 4. Save
    print(f"   Retained {len(final_points)} / {len(stacked_points)} points.")
    save_pcd(final_points, output_file)
    messagebox.showinfo("Success",
                        f"Extraction Complete!\n\nSaved to:\n{output_file}\n\nPoints remaining: {len(final_points)}")


# =====================================================
# GUI MAIN
# =====================================================
def main():
    # Create hidden root window
    root = tk.Tk()
    root.withdraw()  # Hide the main small window

    print("--- 3D-DSF BACKGROUND FILTER TOOL ---")

    # 1. Select Zero Plane File
    print("waiting for user to select Zero Plane file...")
    zero_plane_path = filedialog.askopenfilename(
        title="Select ZERO PLANE File (Background)",
        filetypes=[("PCD Files", "*.pcd"), ("All Files", "*.*")]
    )
    if not zero_plane_path:
        print("❌ Selection cancelled.")
        return

    # 2. Select Stacked Reader File
    print("Waiting for user to select STACKED DATA file...")
    stacked_path = filedialog.askopenfilename(
        title="Select STACKED SCAN File (Raw Data)",
        filetypes=[("PCD Files", "*.pcd"), ("All Files", "*.*")]
    )
    if not stacked_path:
        print("❌ Selection cancelled.")
        return

    # 3. Select Output Location
    print("Waiting for user to choose save location...")
    output_path = filedialog.asksaveasfilename(
        title="Save Filtered Output As...",
        defaultextension=".pcd",
        filetypes=[("PCD Files", "*.pcd")],
        initialfile="filtered_object.pcd"
    )
    if not output_path:
        print("❌ Save cancelled.")
        return

    # 4. Run Filter
    perform_filtering(zero_plane_path, stacked_path, output_path)


if __name__ == "__main__":
    main()