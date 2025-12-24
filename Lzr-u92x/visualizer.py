import open3d as o3d
import numpy as np
import glob
import os
import tkinter as tk
from tkinter import filedialog


class MinimalLiDARViewer:
    def __init__(self):
        # 1. Pick one PCD file
        selected_path = self.pick_file()
        if not selected_path:
            return

        # 2. Auto-detect folder and all PCD files
        directory = os.path.dirname(selected_path)
        self.files = sorted(glob.glob(os.path.join(directory, "*.pcd")))

        if not self.files:
            print(" No .pcd files found.")
            return

        selected_path = os.path.normpath(selected_path)
        self.files = [os.path.normpath(f) for f in self.files]

        try:
            self.index = self.files.index(selected_path)
        except ValueError:
            self.index = 0

        print(f" Loaded folder with {len(self.files)} scans")
        print(f" Starting with: {os.path.basename(self.files[self.index])}")

        # 3. Open3D visualizer
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window(
            window_name="LZR-U921 | 2D LiDAR Viewer",
            width=1024,
            height=768
        )

        # 4. Render options (good for U921)
        opt = self.vis.get_render_option()
        opt.background_color = np.asarray([0, 0, 0])  # black
        opt.point_size = 2.0
        opt.show_coordinate_frame = False

        # 5. Load first scan
        self.pcd = o3d.io.read_point_cloud(self.files[self.index])
        self.prepare_u921_pcd(self.pcd)
        self.vis.add_geometry(self.pcd)

        # 6. Set fixed TOP-DOWN view (very important)
        self.set_top_down_view()

        # 7. Keyboard controls
        self.vis.register_key_callback(262, self.load_next)  # →
        self.vis.register_key_callback(263, self.load_prev)  # ←

        print("\n🎮 CONTROLS")
        print("   →  Next scan")
        print("   ←  Previous scan")
        print("   Mouse : Pan / Zoom")

        self.vis.run()
        self.vis.destroy_window()

    # -----------------------------
    def pick_file(self):
        root = tk.Tk()
        root.withdraw()
        return filedialog.askopenfilename(
            title="Select LZR-U921 PCD file",
            filetypes=[("PCD files", "*.pcd")]
        )

    # -----------------------------
    def prepare_u921_pcd(self, pcd):
        """
        LZR-U921 is a 2D LiDAR → force Z = 0
        """
        points = np.asarray(pcd.points)
        if points.size == 0:
            return

        points[:, 2] = 0.0
        pcd.points = o3d.utility.Vector3dVector(points)

        # Cyan color for visibility
        colors = np.zeros((points.shape[0], 3))
        colors[:, 1] = 1.0
        colors[:, 2] = 1.0
        pcd.colors = o3d.utility.Vector3dVector(colors)

    # -----------------------------
    def set_top_down_view(self):
        """
        Locks bird’s-eye view for 2D LiDAR
        """
        ctr = self.vis.get_view_control()
        ctr.set_front([0, 0, 1])   # look down
        ctr.set_lookat([0, 0, 0])
        ctr.set_up([0, 1, 0])
        ctr.set_zoom(0.8)

    # -----------------------------
    def update_geometry(self):
        new_file = self.files[self.index]
        print(f"➡ {os.path.basename(new_file)}")

        new_pcd = o3d.io.read_point_cloud(new_file)
        self.prepare_u921_pcd(new_pcd)

        self.pcd.points = new_pcd.points
        self.pcd.colors = new_pcd.colors

        self.vis.update_geometry(self.pcd)
        self.vis.poll_events()
        self.vis.update_renderer()

    # -----------------------------
    def load_next(self, vis):
        if self.index < len(self.files) - 1:
            self.index += 1
            self.update_geometry()
        else:
            print(" Last scan")

    def load_prev(self, vis):
        if self.index > 0:
            self.index -= 1
            self.update_geometry()
        else:
            print(" First scan")


if __name__ == "__main__":
    MinimalLiDARViewer()