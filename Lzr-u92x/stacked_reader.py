import serial
import math
import time
import struct
import sys

from serial import SerialException

# --- CONFIGURATION ---
PORT = 'COM4'  # Check Device Manager!
BAUD = 460800  # Standard LZR-U921 Speed
SCAN_SPEED = 0.05  # Speed of movement (meters/second). 0.05 = 5 cm/s (Slow hand lift)


# ---------------------

def save_final_pcd(points, filename):
    """Writes the accumulated 3D points to a single file"""
    if len(points) == 0:
        print("⚠️ No  data collected. File not saved.")
        return

    print(f"\n💾 Saving {len(points)} points to '{filename}'...")

    header = f"""# .PCD v0.7 - LZR 3D Scan
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
    try:
        with open(filename, 'w') as f:
            f.write(header)
            for p in points:
                f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
        print(f"✅ Success! Open '{filename}' in your visualizer.")
    except Exception as e:
        print(f"❌ Error saving file: {e}")


def parse_lzr_packet(packet):
    """Decodes LZR U921 raw bytes into (x, y) coordinates"""
    try:
        # LZR Protocol: Header(2) + MsgID(1) + Size(1) + ...
        # LZR data typically starts after byte 10
        # This is a simplified parser based on standard LZR raw dumps

        # 1. Parse Header Info
        lsn = packet[3]  # Sample count
        if lsn == 0: return []

        # 2. Extract Angles (FSA/LSA)
        fsa = struct.unpack('<H', packet[4:6])[0]
        lsa = struct.unpack('<H', packet[6:8])[0]

        start_angle = (fsa >> 1) / 64.0
        end_angle = (lsa >> 1) / 64.0

        # Handle wrap-around
        angle_diff = end_angle - start_angle
        if angle_diff < 0: angle_diff += 360

        angle_step = angle_diff / (lsn - 1) if lsn > 1 else 0

        raw_points_2d = []

        # 3. Parse Distances
        for i in range(lsn):
            # Distance is at offset 10 + (i * 2)
            base_idx = 10 + (i * 2)
            if base_idx + 2 > len(packet): break

            dist_mm = struct.unpack('<H', packet[base_idx:base_idx + 2])[0]

            # Filter valid range (LZR max is usually ~10m or 65m depending on mode)
            if 20 < dist_mm < 60000:
                dist_m = dist_mm / 1000.0  # Convert mm to Meters

                # Calculate Angle
                current_angle = math.radians(start_angle + (i * angle_step))

                # Convert Polar to Cartesian (X, Y)
                x = dist_m * math.cos(current_angle)
                y = dist_m * math.sin(current_angle)

                raw_points_2d.append((x, y))

        return raw_points_2d

    except Exception as e:
        # print(f"Parse Error: {e}") # Uncomment for debugging
        return []


# --- MAIN LOGIC ---
if __name__ == "__main__":
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        print(f"🚀 Connected to LZR-U921 on {PORT} @ {BAUD}")
        print("------------------------------------------------")
        print(f"👉 MOVE THE SENSOR STEADILY at {SCAN_SPEED * 100} cm/s")
        print("👉 Press Ctrl+C to STOP and SAVE the 3D model.")
        print("------------------------------------------------")

        buffer = b''
        all_3d_points = []
        start_time = time.time()
        packet_count = 0

        while True:
            if ser.in_waiting:
                buffer += ser.read(ser.in_waiting)

                # Process buffer for Header 0x55 0xAA
                while len(buffer) > 10:
                    # Search for header
                    if buffer[0] == 0x55 and buffer[1] == 0xAA:

                        # 1. Determine Packet Size
                        # Byte 3 is often sample count (LSN). Size = 10 header + (LSN * 2) data + 2 checksum
                        lsn = buffer[3]
                        packet_len = 12 + (2 * lsn)  # Approx length formula for LZR

                        if len(buffer) < packet_len:
                            break  # Wait for full packet

                        # 2. Extract Packet
                        packet = buffer[:packet_len]
                        buffer = buffer[packet_len:]  # Remove from buffer

                        # 3. Decode 2D Points
                        points_2d = parse_lzr_packet(packet)

                        if points_2d:
                            # 4. Apply 3D Shift (Time = Z dimension)
                            elapsed = time.time() - start_time
                            z_shift = elapsed * SCAN_SPEED

                            # Add to master list
                            for (x, y) in points_2d:
                                all_3d_points.append((x, y, z_shift))

                            packet_count += 1
                            if packet_count % 10 == 0:
                                sys.stdout.write(
                                    f"\r⏳ Scanning... {len(all_3d_points)} points collected | Height: {z_shift:.2f}m")
                                sys.stdout.flush()

                    else:
                        # If header not found, slide buffer by 1 byte to keep searching
                        buffer = buffer[1:]

    except SerialException as e:
        print(f"\n❌ Port Error: {e}")
        print("Check if another script is using the COM port!")

    except KeyboardInterrupt:
        print("\n\n🛑 Scanning Stopped by User.")
        # Generate Filename with Timestamp
        final_filename = f"LZR_3D_SCAN_{time.strftime('%H%M%S')}.pcd"
        save_final_pcd(all_3d_points, final_filename)

    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("🔌 Connection closed.")