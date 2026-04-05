import serial
import time
import sys

# Default native-USB port for ESP32-S3 on Linux.
PORT = '/dev/ttyACM0'
BAUDRATE = 115200

def send_tx_command():
    if len(sys.argv) != 2 or sys.argv[1] not in ("1000", "2000"):
        raise SystemExit("usage: send_i2c_speed.py 1000|2000 (stop TX first)")
    try:
        # Initialize serial connection
        with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
            print(f"Connected to {PORT}")

            # Flush buffers
            ser.reset_input_buffer()
            ser.reset_output_buffer()

            command = "*i%s*" % sys.argv[1]
            ser.write(command.encode('utf-8'))

            # Wait a moment for the ESP32-S3 to process and respond
            time.sleep(0.1)

            # Read response if any
            if ser.in_waiting:
                response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                print(f"Response from ESP32-S3: {response.strip()}")
            else:
                print("No response received (but command was sent).")

    except serial.SerialException as e:
        print(f"Error: Could not open serial port {PORT}. {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

if __name__ == "__main__":
    send_tx_command()
