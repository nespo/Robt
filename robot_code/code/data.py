import serial
import time

# Setup serial connection
# Define the serial port and baud rate
serial_port = '/dev/ttyUSB0'  # Change this to match the port of your device
baud_rate = 9600  # Change this to match the baud rate of your device

# Open the serial port
ser = serial.Serial(serial_port, baud_rate)

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