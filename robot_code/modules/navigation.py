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
                process_line(line.strip())

def process_line(line):
    if 'Time' in line:
        line = line.split(';', 1)[-1].strip()
    print("Processed Line:", line)  # Debug output

    # Improved GPS handling with separate processing for concatenated sentences
    if '$' in line:
        gps_sentences = re.split('(?=\$)', line)  # Split but keep the delimiter
        for sentence in gps_sentences:
            if sentence.strip():
                parse_gps_data(sentence.strip())
    else:
        if any(x in line for x in ['Roll', 'Pitch', 'Yaw']):
            update_imu_data(line)
        elif 'Error' in line:
            update_error_data(line)

def parse_gps_data(line):
    if valid_nmea_sentence(line):
        if '$GPRMC' in line:
            handle_gprmc(line)
        elif '$GPGGA' in line:
            handle_gpgga(line)
    '''else:
        print(f"Invalid NMEA sentence: {line}")'''

def valid_nmea_sentence(nmea_sentence):
    try:
        data, checksum = nmea_sentence.split('*')
        calculated_checksum = 0
        for char in data[1:]:  # Skip the initial '$'
            calculated_checksum ^= ord(char)
        is_valid = hex(calculated_checksum)[2:].upper() == checksum.upper()
        if not is_valid:
            print(f"Checksum mismatch: Calculated {hex(calculated_checksum)[2:].upper()}, Expected {checksum.upper()}")
            pass
        return is_valid
    except ValueError:
        print(f"Failed to split sentence for checksum: {nmea_sentence}")
        return False

def handle_gprmc(sentence):
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
    try:
        roll, pitch, yaw = map(float, re.findall(r"[-\d.]+", line))
        with data_lock:
            current_orientation['roll'] = roll
            current_orientation['pitch'] = pitch
            current_orientation['yaw'] = yaw
    except ValueError as e:
        print(f"IMU data format error: {e}")
        pass

def update_error_data(line):
    errors = {k: float(v) for k, v in re.findall(r"(\w+): (-?\d+\.\d+)", line)}
    with data_lock:
        current_errors.update(errors)

def convert_to_decimal(degrees_minutes, direction):
    degrees = int(degrees_minutes[:-7])
    minutes = float(degrees_minutes[-7:])
    decimal = degrees + (minutes / 60)
    if direction in ['S', 'W']:
        decimal = -decimal
    return decimal

def get_current_gps():
    while True:  # Loop indefinitely until GPS data is available
        time.sleep(2)  # Wait a bit before checking again to reduce CPU usage
        with data_lock:  # Assuming 'data_lock' is a threading.Lock() to synchronize access to 'current_gps'
            if 'latitude' in current_gps and 'longitude' in current_gps:
                lat = current_gps.get('latitude')
                lon = current_gps.get('longitude')
                if lat is not None and lon is not None:
                    print(f"GPS Data Retrieved: Latitude = {lat}, Longitude = {lon}")
                    return lat, lon
            print("Waiting for valid GPS data...")

def get_current_orientation():
    while True:
        with data_lock:
            if 'yaw' in current_orientation and current_orientation['yaw'] is not None:
                print(f"Orientation Data Retrieved: Yaw = {current_orientation['yaw']}")
                return current_orientation.copy()
            print("Waiting for valid orientation data...")
        time.sleep(1)  # Pause to avoid high CPU usage


def get_current_errors():
    with data_lock:
        return current_errors.copy()

def get_current_heading():
    while True:
        with data_lock:
            yaw = current_orientation.get('yaw')
            if yaw is not None:
                print(f"Heading Data Retrieved: Yaw = {yaw}")
                return yaw
            print("Waiting for valid heading data...")
        time.sleep(1)

print(get_current_gps())
print(get_current_errors)

# Start the serial reading thread
thread = threading.Thread(target=read_serial_data)
thread.daemon = True
thread.start()
