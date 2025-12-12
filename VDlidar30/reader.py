import serial
import math
import time
import struct


PORT = 'COM3'
BAUD = 512000
DURATION = 15.0
SCAN_SPEED = 0.05



def save_3d_pcd(points, filename):
    header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH {len(points)}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {len(points)}
DATA ascii
"""
    with open(filename, 'w') as f:
        f.write(header)
        for p in points:
            # Save X, Y, Z (Space separated)
            f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
    print(f"✅ Saved 3D Model: {filename} ({len(points)} points)")


def parse_packet(packet):
    """Decodes packet to list of (angle, dist_meters)"""
    try:
        lsn = packet[3]
        if lsn <= 0: return []

        fsa = struct.unpack('<H', packet[4:6])[0]
        lsa = struct.unpack('<H', packet[6:8])[0]

        angle_fsa = (fsa >> 1) / 64.0
        angle_lsa = (lsa >> 1) / 64.0

        diff = angle_lsa - angle_fsa
        if diff < 0: diff += 360
        angle_step = diff / (lsn - 1) if lsn > 1 else 0

        raw_data = []
        for i in range(lsn):
            idx = 10 + (2 * i)
            dist_val = struct.unpack('<H', packet[idx:idx + 2])[0]
            dist_m = dist_val / 4000.0  # TG30 Unit Conversion

            if dist_m > 0:
                current_angle = angle_fsa + (angle_step * i)
                raw_data.append((current_angle, dist_m))
        return raw_data
    except:
        return []


# --- MAIN LOOP ---
try:
    ser = serial.Serial(PORT, BAUD, timeout=1)

    # Wake Up
    ser.setDTR(True)
    ser.write(b'\xA5\x60')
    time.sleep(0.5)
    ser.reset_input_buffer()

    print(f"🎥 STARTING 3D SCAN ({DURATION}s)...")
    print(f"👉 INSTRUCTION: Move the LiDAR steadily UPWARDS or FORWARD at {SCAN_SPEED * 100} cm/s")

    buffer = b''
    fused_cloud = []

    start_time = time.time()

    while (time.time() - start_time) < DURATION:
        if ser.in_waiting:
            buffer += ser.read(ser.in_waiting)

            while len(buffer) > 10:
                if buffer[0] == 0xAA and buffer[1] == 0x55:
                    lsn = buffer[3]
                    packet_len = 10 + (2 * lsn)

                    if len(buffer) < packet_len:
                        break

                    packet = buffer[:packet_len]
                    buffer = buffer[packet_len:]

                    # 1. Get raw 2D slice
                    slice_data = parse_packet(packet)

                    # 2. Calculate "Z-Shift" (or X-Shift) based on Time
                    # This turns time into a dimension
                    elapsed = time.time() - start_time
                    shift_amount = elapsed * SCAN_SPEED

                    for (angle_deg, r) in slice_data:
                        angle_rad = math.radians(angle_deg)

                        # --- COORDINATE MAPPING ---
                        # Imagine the LiDAR is flat on a table (X, Y are the circle)
                        # We are lifting it UP (Z-axis)

                        x = r * math.cos(angle_rad)
                        y = r * math.sin(angle_rad)
                        z = shift_amount  # The vertical movement

                        fused_cloud.append((x, y, z))
                else:
                    buffer = buffer[1:]

    print("🛑 Scan Complete.")
    ser.write(b'\xA5\x65')
    ser.close()

    # Save file
    filename = f"object_scan_{int(time.time())}.pcd"
    save_3d_pcd(fused_cloud, filename)

except KeyboardInterrupt:
    print("Aborted.")
    if 'ser' in locals(): ser.close()