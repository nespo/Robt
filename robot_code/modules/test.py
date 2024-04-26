import serial
import time

# Setup serial connection
ser = serial.Serial('/dev/ttyUSB0', 256000, timeout=1)

# Attempt to read data
try:
    print("Starting to read from serial port...")
    time.sleep(2)  # Give it a moment before reading
    while True:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            print('Received:', data)
        time.sleep(1)
except KeyboardInterrupt:
    print("Program interrupted by user.")
finally:
    ser.close()
    print("Serial port closed.")
