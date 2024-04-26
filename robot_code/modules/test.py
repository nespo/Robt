import serial

ser = serial.Serial(port="/dev/ttyUSB0", baudrate=256000)
try:
    while True:
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            print("Received:", data)
except KeyboardInterrupt:
    print("Stopped by user")
finally:
    ser.close()
    print("Serial port closed")
