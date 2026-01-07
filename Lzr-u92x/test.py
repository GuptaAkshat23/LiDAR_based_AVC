import serial
import struct
import time
import sys

# =====================================================
# CONFIGURATION
# =====================================================
SERIAL_PORT = '/dev/ttyUSB0'  # Ensure this matches your setup
BAUD_RATE = 921600  # Matching your LZR-u921 config


def run_frequency_check():
    try:
        # Open Serial Port
        # increased buffer size to prevent overflow during testing
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
        ser.reset_input_buffer()
        print(f"🔌 Connected to {SERIAL_PORT} @ {BAUD_RATE}")
        print("📊 Measuring Scan Frequency... (Press Ctrl+C to stop)\n")
    except Exception as e:
        print(f"❌ Could not open port: {e}")
        return

    frame_count = 0
    start_time = time.time()

    # We track the serial buffer to see if the Pi is choking
    max_buffer_usage = 0

    try:
        while True:
            # 1. Check Serial Buffer Health
            waiting_bytes = ser.in_waiting
            max_buffer_usage = max(max_buffer_usage, waiting_bytes)

            # 2. Attempt to read the Header (The "Bottleneck" Check)
            # This logic mimics your production code to test ITS speed.
            if ser.read(1) == b'\xfc':
                if ser.read(3) == b'\xfd\xfe\xff':
                    # Header Found! Now read size
                    size_data = ser.read(2)
                    if len(size_data) < 2: continue

                    size = struct.unpack('<H', size_data)[0]

                    # Read the rest of the message (Body + Checksum)
                    # We read 'size' bytes.
                    # Your original code reads 'size' then reads 2 more for checksum?
                    # Let's align exactly with your provided logic:
                    # "body = self.ser.read(size); self.ser.read(2)"

                    body = ser.read(size)
                    checksum = ser.read(2)

                    if len(body) == size and len(checksum) == 2:
                        # Valid Frame Captured
                        frame_count += 1

            # 3. Calculate and Print Frequency every 1.0 second
            current_time = time.time()
            elapsed = current_time - start_time

            if elapsed >= 1.0:
                hz = frame_count / elapsed

                # Visual Indicator
                status = "✅ GOOD" if hz > 55 else "⚠️ SLOW" if hz > 30 else "❌ CRITICAL"

                print(f"Rate: {hz:5.2f} Hz | Buffer: {waiting_bytes:4} bytes | {status}")

                # Reset counters
                frame_count = 0
                start_time = current_time
                max_buffer_usage = 0

    except KeyboardInterrupt:
        print("\n\n🛑 Test Stopped.")
        ser.close()


if __name__ == "__main__":
    run_frequency_check()