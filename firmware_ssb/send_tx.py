import serial
import time
import sys

# Default port for Pico on Linux. Change if needed (e.g., 'COM3' on Windows)
PORT = '/dev/ttyACM0'
BAUDRATE = 115200

def send_tx_command():
    try:
        # Initialize serial connection
        with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
            print(f"Connected to {PORT}")

            # Flush buffers
            ser.reset_input_buffer()
            ser.reset_output_buffer()

            # Send the start_tx CAT command (now a toggle)
            command = "*t*"
            print(f"Sending toggle command: {command}")
            ser.write(command.encode('utf-8'))

            # Wait a moment for the Pico to process and respond
            time.sleep(0.1)

            # Read response if any
            if ser.in_waiting:
                response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                print(f"Response from Pico: {response.strip()}")
            else:
                print("No response received (but command was sent).")

    except serial.SerialException as e:
        print(f"Error: Could not open serial port {PORT}. {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

if __name__ == "__main__":
    send_tx_command()
