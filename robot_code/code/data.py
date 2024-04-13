import serial
import time

# Define the serial port and baud rate
serial_port = '/dev/ttyACM0'  # Change this to match the port of your Arduino Uno
baud_rate = 9600  # Change this to match the baud rate of your Arduino Uno

# Open the serial port
ser = serial.Serial(serial_port, baud_rate)

try:
    # Read data from the serial port
    while True:
        line = ser.readline().decode('utf-8').rstrip()  # Read a line of data
        print("Received:", line)
        # Add your processing logic here

except KeyboardInterrupt:
    # Close the serial port when the program is interrupted
    ser.close()
