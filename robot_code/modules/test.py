import serial
import time

# Setup serial connection
ser = serial.Serial('/dev/ttyUSB0', 256000, timeout=1)

try:
    print("Starting to read from serial port...")
    time.sleep(2)  # Give it a moment before reading
    while True:
        data = ser.read(100)  # Attempt to read 100 bytes
        if data:
            print('Received:', data)
        else:
            print('No data received.')
        time.sleep(1)
except KeyboardInterrupt:
    print("Program interrupted by user.")
finally:
    ser.close()
    print("Serial port closed.")
