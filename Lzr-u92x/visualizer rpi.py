import open3d as o3d
import numpy as np
import glob
import os
import tkinter as tk
from tkinter import filedialog
import matplotlib.pyplot as plt

# Check if we can open a display (headless check)
if os.environ.get('DISPLAY', '') == '':
    print('no display found. Using :0.0')
    os.environ.__setitem__('DISPLAY', ':0.0')


class PiLiDARViewer:
    def __init__(self):
        # 1. Open File Dialog
        selected_path = self.pick_file()
        if not selected_path:
            return

        # 2. Smart Folder Detection
        directory = os.path.dirname(selected_path)
        search_pattern = os.path.join(directory, "*.pcd")
        self.files = sorted(glob.glob(search_pattern))

        if not self.files:
            print(" No files found.")
            return

        # 3. Find index
        selected_path = os.path.normpath(selected_path)
        self.files = [os.path.normpath(f) for f in self.files]
        try:
            self.index = self.files.index(selected_path)
        except ValueError:
            self.index = 0

        print(f"📂 Visualizing: {os.path.basename(self.files[self.index])}")

        # 4. Setup Matplotlib Figure (Lighter than Open3D Visualizer)
        self.fig = plt.figure(figsize=(10, 7))
        self.ax = self.fig.add_subplot(111, projection='3d')

        # Set background to black for that "LiDAR look"
        self.fig.patch.set_facecolor('black')
        self.ax.set_facecolor('black')
        self.ax.grid(False)
        self.ax.axis('off')  # Hide axes for clean look

        # 5. Connect Key Events
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)

        print("\n🎮 CONTROLS:")
        print("   [Right Arrow] : Next Scan")
        print("   [Left Arrow]  : Previous Scan")

        # 6. Render First Frame
        self.update_plot()
        plt.show()

    def pick_file(self):
        root = tk.Tk()
        root.withdraw()
        file_path = filedialog.askopenfilename(
            title="Select a LiDAR Scan (.pcd)",
            filetypes=[("Point Cloud Data", "*.pcd")]
        )
        return file_path

    def update_plot(self):
        filename = self.files[self.index]
        print(f"➡️ Loading: {os.path.basename(filename)}")

        # Read with Open3D (It's good at reading!)
        pcd = o3d.io.read_point_cloud(filename)

        # Convert to numpy for Matplotlib
        points = np.asarray(pcd.points)

        # --- OPTIMIZATION FOR RASPBERRY PI ---
        # Plotting 100k+ points will freeze the Pi.
        # We skip points (downsample) to make it run smooth.
        # [::10] means take every 10th point. Change to [::5] for more detail.
        skip = 10
        points = points[::skip]

        self.ax.clear()
        self.ax.axis('off')  # Keep it clean after clear

        # Scatter Plot (White/Cyan points on Black)
        # s=0.5 is point size (small for detail)
        self.ax.scatter(points[:, 0], points[:, 1], points[:, 2], s=0.5, c='cyan', marker='.')

        self.fig.canvas.draw()

    def on_key(self, event):
        if event.key == 'right':
            if self.index < len(self.files) - 1:
                self.index += 1
                self.update_plot()
            else:
                print("End of list.")
        elif event.key == 'left':
            if self.index > 0:
                self.index -= 1
                self.update_plot()
            else:
                print("Start of list.")


if __name__ == "__main__":
    PiLiDARViewer()