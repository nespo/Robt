import re
import threading
import serial
import time

# Setup serial connection
serial_port = '/dev/ttyACM0'  # Update to match your device port in Windows
baud_rate = 19200
ser = serial.Serial(serial_port, baud_rate, timeout=1)

# Shared structures to hold GPS and IMU data
current_gps = {}
current_orientation = {}

# Lock for thread-safe access to data
data_lock = threading.Lock()

def read_serial_data():
    buffer = ""
    while True:
        try:
            data = ser.read(1)  # Read one byte at a time
            if data:
                buffer += data.decode('utf-8', errors='ignore')
                if '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    #print("Received:", line)  # Debug line
                    process_line(line.strip())
        except serial.SerialException as e:
            print(f"Serial error: {e}")
            break

def process_line(line):
    if 'Latitude' in line or 'Longitude' in line:
        update_gps_data(line)
    elif any(k in line for k in ['Roll', 'Pitch', 'Yaw']):
        update_imu_data(line)

def update_gps_data(line):
    try:
        gps_data = {k: float(v) for k, v in re.findall(r'(Latitude|Longitude):([\d.]+)', line)}
        with data_lock:
            current_gps.update(gps_data)
    except ValueError:
        print(f"Invalid GPS data in line: {line}")

def update_imu_data(line):
    #print("Trying to update IMU with line:", line)  # Debug to check what is being processed
    try:
        imu_data = {k: float(v) for k, v in re.findall(r'(\w+):\s*(-?\d+\.\d+)', line)}
        if imu_data:
            with data_lock:
                current_orientation.update(imu_data)
            #print(f"Updated IMU Data: {imu_data}")  # Debug for successful update
        else:
            print("No IMU data extracted from line.")  # Debug for failed regex match
    except ValueError as e:
        print(f"Invalid IMU data in line: {line} with error {e}")


def get_current_gps():
    while True:  # Loop indefinitely until GPS data is available
        time.sleep(2) 
        print("Trying to get GPS data: ", current_gps) # Wait a bit before checking again to reduce CPU usage
        with data_lock:  # Assuming 'data_lock' is a threading.Lock() to synchronize access to 'current_gps'
            if 'Latitude' in current_gps and 'Longitude' in current_gps:
                lat = current_gps.get('Latitude')
                lon = current_gps.get('Longitude')
                if lat is not None and lon is not None:
                    print(f"GPS Data Retrieved: Latitude = {lat}, Longitude = {lon}")
                    return lat, lon
            #print("Waiting for valid GPS data...")

def get_current_orientation():
    while True:
        with data_lock:
            if 'Yaw' in current_orientation and current_orientation['Yaw'] is not None:
                print(f"Orientation Data Retrieved: Yaw = {current_orientation['Yaw']}")
                return current_orientation.copy()
            #print("Waiting for valid orientation data...")
        time.sleep(1)  # Pause to avoid high CPU usage
    
def get_current_heading():
    while True:
        with data_lock:
            yaw = current_orientation.get('Yaw')
            if yaw is not None:
                print(f"Heading Data Retrieved: Yaw = {yaw}")
                return yaw
            #print("Waiting for valid heading data...")
        time.sleep(1)


# Thread to handle serial data reading
thread = threading.Thread(target=read_serial_data)
thread.daemon = True
thread.start()

'''if __name__ == "__main__":
    while True:
        gps = get_current_gps()
        orientation = get_current_heading()
        if gps:
            print("GPS:", gps)
        if orientation:
            print("Orientation:", orientation)
        if not gps and not orientation:
            print("Waiting for new data...")
        time.sleep(1)'''
