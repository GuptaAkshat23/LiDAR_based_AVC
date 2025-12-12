import matplotlib.pyplot as plt
import os
import tkinter as tk
from tkinter import filedialog


# ==========================================
# FILE SELECTION GUI
# ==========================================
def select_file():
    """Opens a system file explorer to pick a .pcd file."""
    # Create a hidden root window for the dialog
    root = tk.Tk()
    root.withdraw()

    # Set default directory to './scans' if it exists, otherwise current folder
    start_dir = "./scans" if os.path.exists("./scans") else "."

    print("Opening file selector...")
    file_path = filedialog.askopenfilename(
        title="Select a LZR-U921 Scan File",
        initialdir=start_dir,
        filetypes=[("PCD Files", "*.pcd"), ("All Files", "*.*")]
    )

    if not file_path:
        print("No file selected. Exiting.")
        return None

    return file_path


# ==========================================
# PCD PARSING LOGIC
# ==========================================
def read_pcd_data(filename):
    """Reads X and Y coordinates from a standard ASCII PCD file."""
    x_points = []
    y_points = []

    try:
        with open(filename, 'r') as f:
            lines = f.readlines()

            # Locate the start of the data
            data_start_index = 0
            for i, line in enumerate(lines):
                if line.strip().startswith("DATA ascii"):
                    data_start_index = i + 1
                    break

            # Parse coordinates
            for i in range(data_start_index, len(lines)):
                parts = lines[i].strip().split()
                # Ensure we have at least X and Y
                if len(parts) >= 2:
                    try:
                        x = float(parts[0])
                        y = float(parts[1])

                        # Filter out 0,0 points (often sensor errors or max range)
                        if not (x == 0.0 and y == 0.0):
                            x_points.append(x)
                            y_points.append(y)
                    except ValueError:
                        continue

        return x_points, y_points
    except Exception as e:
        print(f"Error reading file: {e}")
        return [], []


# ==========================================
# PLOTTING LOGIC
# ==========================================
def visualize_scan(filename):
    x, y = read_pcd_data(filename)

    if not x:
        print("File is empty or could not be read.")
        return

    # Create the Plot
    plt.figure(figsize=(10, 8))

    # 1. Plot the Scan Points (Blue Dots)
    plt.plot(x, y, 'bo', markersize=3, label='Lidar Return')

    # 2. Plot the Sensor Location (Red X at 0,0)
    plt.plot(0, 0, 'rx', markersize=12, markeredgewidth=3, label='LZR U921 Sensor')

    # Formatting
    plt.title(f"Scan View: {os.path.basename(filename)}")
    plt.xlabel("X Distance (meters) - [Left/Right]")
    plt.ylabel("Y Distance (meters) - [Forward]")
    plt.legend()
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.axis('equal')  # Critical: Keeps the aspect ratio 1:1

    # Show
    print(f"Displaying {len(x)} points...")
    plt.show()


if __name__ == "__main__":
    # 1. Ask user to pick a file
    target_file = select_file()

    # 2. If they picked one, show it
    if target_file:
        visualize_scan(target_file)