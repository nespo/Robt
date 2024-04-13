import serial
import time

# Setup serial connection
serial_port = '/dev/ttyACM0'  # Change this to match the port of your Arduino Uno
baud_rate = 19200
ser = serial.Serial(serial_port, baud_rate, timeout=1)

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
