import re
import threading
import serial
import time

# Setup serial connection
serial_port = '/dev/ttyACM0'  # Change this to match the port of your Arduino Uno
baud_rate = 19200
ser = serial.Serial(serial_port, baud_rate, timeout=1)

# Shared structures to hold GPS and IMU data
current_gps = {}
current_orientation = {}
current_errors = {}

# Lock for thread-safe access to data
data_lock = threading.Lock()

def read_serial_data():
    buffer = ""
    while True:
        data = ser.read(1)  # Read one byte at a time
        if data:
            buffer += data.decode('utf-8', errors='ignore')
            if '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                print(f"Read line: {line.strip()}")  # Debug output
                process_line(line.strip())

def process_line(line):
    print(f"Processing line: {line}")  # Debug output

    # Check for IMU data first, if no IMU data then check for GPS data
    if 'Roll' in line or 'Pitch' in line or 'Yaw' in line:
        update_imu_data(line)
    else:
        # This will ignore anything before the first '$' character
        gps_data = re.search(r"\$(.*)", line)
        if gps_data:
            gps_sentences = re.split('(?=\$)', gps_data.group(0))  # Split but keep the delimiter
            for sentence in gps_sentences:
                if sentence.strip():
                    parse_gps_data(sentence.strip())



def update_imu_or_error_data(part):
    if any(x in part for x in ['Roll', 'Pitch', 'Yaw']):
        update_imu_data(part)
    elif 'Error' in part:
        update_error_data(part)

def parse_gps_data(line):
    print(f"Parsing GPS data: {line}")  # Debug output
    if valid_nmea_sentence(line):
        if '$GPRMC' in line:
            handle_gprmc(line)
        elif '$GPGGA' in line:
            handle_gpgga(line)

def valid_nmea_sentence(nmea_sentence):
    try:
        # Ignore any non-NMEA prefix data
        if not nmea_sentence.startswith('$'):
            return False

        data, checksum = nmea_sentence.split('*')
        calculated_checksum = 0
        for char in data[1:]:  # Skip the initial '$'
            calculated_checksum ^= ord(char)
        is_valid = hex(calculated_checksum)[2:].upper().zfill(2) == checksum.upper()
        if not is_valid:
            print(f"Checksum mismatch: Calculated {hex(calculated_checksum)[2:].upper()}, Expected {checksum.upper()}")
        return is_valid
    except ValueError:
        print(f"Failed to split sentence for checksum: {nmea_sentence}")
        return False


def handle_gprmc(sentence):
    print(f"Handling GPRMC: {sentence}")  # Debug output
    parts = sentence.split(',')
    if len(parts) > 8 and parts[2] == 'A':
        latitude = convert_to_decimal(parts[3], parts[4])
        longitude = convert_to_decimal(parts[5], parts[6])
        with data_lock:
            current_gps['latitude'] = latitude
            current_gps['longitude'] = longitude
            current_gps['speed'] = float(parts[7])
            current_gps['date'] = parts[9]

def handle_gpgga(sentence):
    print(f"Handling GPGGA: {sentence}")  # Debug output
    parts = sentence.split(',')
    if len(parts) > 6 and parts[6] != '0':
        latitude = convert_to_decimal(parts[2], parts[3])
        longitude = convert_to_decimal(parts[4], parts[5])
        altitude = f"{parts[9]} {parts[10]}"
        with data_lock:
            current_gps['latitude'] = latitude
            current_gps['longitude'] = longitude
            current_gps['altitude'] = altitude

def update_imu_data(line):
    print(f"Updating IMU data: {line}")  # Debug output
    try:
        imu_parts = re.search(r"Roll:(-?\d+\.\d+); Pitch:(-?\d+\.\d+); Yaw:(-?\d+\.\d+)", line)
        if imu_parts:
            roll, pitch, yaw = map(float, imu_parts.groups())
            with data_lock:
                current_orientation['roll'] = roll
                current_orientation['pitch'] = pitch
                current_orientation['yaw'] = yaw
        else:
            print("IMU data format error: Incorrect number of values or format")
    except ValueError as e:
        print(f"IMU data parsing error: {e}")

def update_error_data(line):
    print(f"Updating error data: {line}")  # Debug output
    errors = re.findall(r"(\w+Error\w+): (-?\d+\.\d+)", line)
    if errors:
        with data_lock:
            for key, value in errors:
                current_errors[key] = float(value)
    else:
        print("Error data format not recognized")


def convert_to_decimal(degrees_minutes, direction):
    degrees = int(degrees_minutes[:-7])
    minutes = float(degrees_minutes[-7:])
    decimal = degrees + (minutes / 60)
    if direction in ['S', 'W']:
        decimal = -decimal
    return decimal


def get_current_gps():
    with data_lock:
        return current_gps.copy()

def get_current_orientation():
    with data_lock:
        return current_orientation.copy()

def get_current_errors():
    with data_lock:
        return current_errors.copy()

# Start the serial reading thread
thread = threading.Thread(target=read_serial_data)
thread.daemon = True
thread.start()
