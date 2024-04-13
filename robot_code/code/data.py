import serial
import time

# Setup serial connection
ser = serial.Serial('COM3', 19200, timeout=1)  # Update the COM port as necessary

def read_serial_data():
    if ser.in_waiting > 0:
        try:
            # Attempt to read and decode the line; ignore errors or replace them
            data = ser.readline().decode('utf-8', errors='replace').strip()
            if data:
                print(data)
        except UnicodeDecodeError as e:
            print(f"Decode error: {e}, data received may be corrupted")

try:
    while True:
        read_serial_data()
        time.sleep(0.1)  # Adjust sleep time based on how frequent you need the data
except KeyboardInterrupt:
    ser.close()
    print("Serial connection closed.")