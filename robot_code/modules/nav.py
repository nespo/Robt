import re
import threading
import serial
import time

# Setup serial connection
serial_port = '/dev/ttyACM0'  # Update to match your device
baud_rate = 19200
ser = serial.Serial(serial_port, baud_rate, timeout=1)

# Shared structures to hold GPS and IMU data
current_gps = {}
current_orientation = {}
current_errors = {}

# Lock for thread-safe access to data
data_lock = threading.Lock()

def read_serial_data():
    """Continuously read data from serial port and process each complete line."""
    buffer = ""
    while True:
        data = ser.read(1)
        if data:
            buffer += data.decode('utf-8', errors='ignore')
            if '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                try:
                    process_line(line.strip())
                except Exception as e:
                    print(f"Error processing line: {line}, Error: {e}")

def process_line(line):
    """Determine the type of data received and update appropriate structures."""
    if 'Latitude' in line or 'Longitude' in line:
        update_gps_data(line)
    elif 'AccError' in line or 'GyroError' in line:
        update_error_data(line)
    elif 'Time' in line:
        update_imu_data(line)

def update_gps_data(line):
    """Parse and update GPS data."""
    try:
        gps_data = {k: float(v) for k, v in re.findall(r'(Latitude|Longitude):(\d+\.\d+)', line)}
        with data_lock:
            current_gps.update(gps_data)
    except ValueError:
        print(f"Invalid GPS data: {line}")

def update_imu_data(line):
    """Extract and store orientation data."""
    try:
        imu_data = {k: float(v) for k, v in re.findall(r'(Roll|Pitch|Yaw):(-?\d+\.\d+)', line)}
        with data_lock:
            current_orientation.update(imu_data)
    except ValueError:
        print(f"Invalid IMU data: {line}")

def update_error_data(line):
    """Extract error data and update the corresponding structure."""
    try:
        error_data = {k: float(v) for k, v in re.findall(r'(AccErrorX|AccErrorY|GyroErrorX|GyroErrorY|GyroErrorZ):(-?\d+\.\d+)', line)}
        with data_lock:
            current_errors.update(error_data)
    except ValueError:
        print(f"Invalid error data: {line}")

def get_current_gps():
    start_time = time.time()
    timeout = 30  # Set a reasonable timeout
    print("Attempting to retrieve GPS data...")
    while True:
        with data_lock:
            if 'Latitude' in current_gps and 'Longitude' in current_gps:
                lat = current_gps.get('Latitude')
                lon = current_gps.get('Longitude')
                if lat is not None and lon is not None:
                    print(f"GPS Data Retrieved: Latitude = {lat}, Longitude = {lon}")
                    return lat, lon
            else:
                print("GPS data not yet available in dictionary.")
            if time.time() - start_time > timeout:
                print("Timeout reached, GPS data not available.")
                break
        print("Waiting for valid GPS data...")
        time.sleep(2)
    return None, None



def get_current_orientation():
    """Fetch the latest orientation data if available."""
    with data_lock:
        return current_orientation.copy()

def get_current_errors():
    """Fetch the latest error metrics."""
    with data_lock:
        return current_errors.copy()
    
def get_current_heading():
    while True:
        with data_lock:
            yaw = current_orientation.get('Yaw')
            if yaw is not None:
                print(f"Heading Data Retrieved: Yaw = {yaw}")
                return yaw
            print("Waiting for valid heading data...")

# Thread to handle serial data reading
thread = threading.Thread(target=read_serial_data)
thread.daemon = True
thread.start()
# Main script execution
if __name__ == '__main__':
    try:
        while True:
            print("GPS Data:", get_current_gps())
            print("Orientation Data:", get_current_orientation())
            print("Error Data:", get_current_errors())
            time.sleep(1)
    except KeyboardInterrupt:
        ser.close()
        print("Serial connection closed.")
