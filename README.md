# 📡 LiDAR-Based Automatic Vehicle Classification (AVC)

**Real-time Vehicle Detection · 3D Point Cloud Processing · Toll Plaza Automation**

This project is a high-performance system designed to detect, track, and classify vehicles using LiDAR technology. Developed for toll plaza automation, it interfaces with industrial LiDAR sensors (LZR-U921 and TG30) to generate accurate 3D point clouds and 2D side-profile images of passing vehicles.

---

## 🛠 Tech Stack

**Core**
- Python 3.8+
- NumPy & SciPy (Data processing)

**3D & Image Processing**
- Open3D (Point cloud manipulation)
- OpenCV (Image generation & filtering)

**Hardware Interface**
- PySerial (UART/USB communication)
- Raspberry Pi / Jetson Nano (Target Platforms)

---

## 📑 Table of Contents

- [Overview](#overview)
- [Key Features](#key-features)
- [Hardware Support](#hardware-support)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
- [Usage](#usage)
  - [LZR-U921 Module](#lzr-u921-module)
  - [VD/TG30 Module](#vdtg30-module)
  - [Visualization](#visualization)
- [Project Structure](#project-structure)
- [Configuration](#configuration)
- [Future Enhancements](#future-enhancements)
- [License](#license)

---

## 📖 Overview

The LiDAR AVC system addresses the need for accurate vehicle profiling in traffic management. Unlike camera-based systems, this project uses laser scanning to create depth-accurate models of vehicles, unaffected by lighting conditions.

It functions by:
1. **Capturing** raw polar data from LiDAR sensors via serial.
2. **Calibrating** a "Zero Plane" to identify the static background (road/ground).
3. **Detecting** moving objects using background subtraction and persistence logic.
4. **Reconstructing** the vehicle in 3D by stacking scan lines over time.
5. **Generating** clean 2D side-view images for classification algorithms.

---

## ✨ Key Features

- ⚡ **Real-Time Threading**
  Decoupled serial acquisition and data processing threads ensure no data loss at high baud rates (up to 921,600).

- 🧹 **Advanced Noise Filtering**
  - **Statistical Outlier Removal:** Eliminates dust and rain noise.
  - **Radius Outlier Removal:** Cleans up flying pixels.
  - **DBSCAN Clustering:** Groups points to isolate the main vehicle object.

- 📐 **Dynamic Calibration**
  Automated "Zero Plane" calibration learns the background environment to accurately subtract the road and static structures.

- 🖼️ **Image Generation**
  Converts 3D point clouds into high-contrast 2D binary images (Side Views) suitable for machine learning classification.

- 🎮 **Universal Visualizer**
  Built-in Open3D-based tool to playback and inspect captured `.pcd` scan files.

---

## 🔌 Hardware Support

The repository contains specialized drivers for two types of sensors:

| Module | Sensor Model | Baud Rate | Application |
| :--- | :--- | :--- | :--- |
| **Lzr-u92x** | BEA LZR-U921 | 921,600 | High-speed side profiling |
| **VDlidar30** | YDLidar TG30 / VD30 | 512,000 | 360° scanning & clustering |

---

## 🚀 Getting Started

### ✅ Prerequisites

- Python 3.x
- Drivers for your USB-to-Serial adapter

### 📥 Installation

1. **Clone the repository**
   ```bash
   git clone [https://github.com/guptaakshat23/lidar_based_avc.git](https://github.com/guptaakshat23/lidar_based_avc.git)
   ```

2. **Navigate to the project directory**
   ```bash
   cd lidar_based_avc
   ```

3. **Install dependencies**
   ```bash
   pip install pyserial numpy opencv-python open3d scipy
   ```
   *(Note: For Raspberry Pi, you may need to install Open3D via pre-built wheels or source).*

---

## ▶️ Usage

### LZR-U921 Module (Side Profiling)
Ideal for generating side-view images of vehicles.

1. **Navigate to the folder:**
   ```bash
   cd Lzr-u92x
   ```
2. **Run the master control script:**
   ```bash
   python master_control.py
   ```
   *The system will calibrate for 4500 frames and then enter "Persistence Mode" to wait for vehicles.*

### VD/TG30 Module (360° Detection)
Uses clustering to detect objects in a 360-degree field of view.

1. **Navigate to the folder:**
   ```bash
   cd VDlidar30
   ```
2. **Run the pipeline:**
   ```bash
   python master_control.py
   ```

### Visualization
To inspect the captured `.pcd` files:

```bash
python VDlidar30/visualizer.py
```
1. Select a `.pcd` file from the dialog.
2. Use **Left/Right Arrow Keys** to navigate through scans.
3. Use **Mouse** to rotate and zoom.

---

## 🗂 Project Structure

```text
lidar_based_avc/
├── Lzr-u92x/                 # Logic for LZR-U921 Sensor
│   ├── master_control.py     # Main entry point (PC/Serial)
│   ├── code.py               # Optimized version for RPi
│   ├── image_generator.py    # Converts PCD to PNG
│   └── calibration/          # Stored background models
├── VDlidar30/                # Logic for TG30/VD30 Sensor
│   ├── master_control.py     # Main pipeline with DBSCAN
│   ├── visualizer.py         # Universal PCD Viewer tool
│   └── scan_data/            # Output folder for scans
└── README.md
```

---

## ⚙️ Configuration

You can tweak system parameters directly in the `master_control.py` files:

**Physics & Geometry:**
```python
SERIAL_PORT = 'COM8'       # or '/dev/ttyUSB0'
VEHICLE_SPEED_KMPH = 25.0  # Used to reconstruct Z-axis
START_ANGLE = -48.0        # Sensor mounting angle
```

**Detection Logic:**
```python
TRIGGER_THRESHOLD = 15     # Points needed to trigger recording
REQUIRED_PERSISTENCE = 3   # Frames to confirm object validity
IDLE_TIMEOUT = 0.5         # Time to wait before saving
```

---

## 🔮 Future Enhancements

- 🧠 **Deep Learning Integration:** Feed generated side-view images directly into a CNN for vehicle class (Car/Truck/Bus) prediction.
- 📡 **MQTT / IoT Support:** Send classification results to a central server or cloud dashboard.
- 🚦 **Speed Estimation:** Dynamic speed calculation instead of fixed `VEHICLE_SPEED_KMPH` for more accurate 3D reconstruction.

---

## 📄 License

This project is available for educational and research purposes.

⭐ **If you find this project useful for your AVC research, give it a star on GitHub!**
