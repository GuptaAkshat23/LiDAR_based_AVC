import serial
import time
import sys

# List of standard LZR-U921 baud rates to test
BAUD_RATES = [460800, 921600, 38400, 115200, 57600, 19200]
PORT = 'COM4'  # Make sure this is correct


def print_hex(data):
    return " ".join(f"{b:02X}" for b in data)


print(f"--- LZR-U921 CONNECTION DIAGNOSTIC ---")
print(f"Target Port: {PORT}")

found_valid_connection = False

for baud in BAUD_RATES:
    print(f"\n[TESTING] Baud Rate: {baud}...")
    try:
        ser = serial.Serial(PORT, baud, timeout=2)

        # Clear buffer
        ser.reset_input_buffer()

        # Listen for 3 seconds
        start_time = time.time()
        buffer = b""
        print("   Listening...", end="", flush=True)

        while (time.time() - start_time) < 3.0:
            if ser.in_waiting:
                chunk = ser.read(ser.in_waiting)
                buffer += chunk
                # If we have enough data, stop early
                if len(buffer) > 100:
                    break
            time.sleep(0.1)

        ser.close()

        # ANALYZE RESULT
        if len(buffer) == 0:
            print(f"\n   -> RESULT: SILENCE (No data received)")
            print("      (Check: Is the Sensor Powered? Are A/B wires swapped?)")
        else:
            print(f"\n   -> RESULT: RECEIVED {len(buffer)} BYTES")
            print(f"      Raw Hex: {print_hex(buffer[:20])} ...")

            # Check for the Magic Signature: FF FE FD FC
            if b'\xff\xfe\xfd\xfc' in buffer:
                print(f"   >>> SUCCESS! VALID LZR DATA FOUND AT {baud} BAUD! <<<")
                found_valid_connection = True
                break
            else:
                print("      -> Garbage data (Wrong Baud Rate or A/B Swapped)")

    except serial.SerialException:
        print(f"\n   [ERROR] Could not open port {PORT}. (Is it open in another program?)")
        break

if not found_valid_connection:
    print("\n--- DIAGNOSIS ---")
    print("1. If ALL speeds showed 'SILENCE': Check your Wiring (A/B) and Power.")
    print("2. If you saw 'Garbage Data' but no success: Try swapping A/B wires.")
    print("3. Ensure you are using a USB-to-RS485 converter, NOT RS232.")
else:
    print("\n--- NEXT STEPS ---")
    print(f"Update your recorder scripts to use BAUD_RATE = {baud}")