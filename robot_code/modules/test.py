import time
from rplidar import RPLidar

# Define the port name for the RPLIDAR
PORT_NAME = '/dev/ttyUSB0'  # Change this to your actual port name

# Function to process scan data
def process_scan_data(scan):
    for angle, distance in scan:
        print(f"Angle: {angle}, Distance: {distance}")

# Connect to the RPLIDAR
lidar = RPLidar(PORT_NAME)

try:
    # Start scanning
    lidar.start_motor()
    lidar.connect()
    
    # Start scanning
    lidar.start_scan()
    
    # Get scan data for 30 seconds
    end_time = time.time() + 30
    while time.time() < end_time:
        # Read scan data
        scan_data = lidar.iter_scans()
        
        # Process the scan data
        for scan in scan_data:
            process_scan_data(scan)
        
finally:
    # Stop scanning and clean up
    lidar.stop_scan()
    lidar.stop_motor()
    lidar.disconnect()
