from robot_code.modules.us import get_current_gps, get_current_errors, get_current_orientation # Make sure the module name matches the file name
from time import sleep

while True:
    # Fetch the latest data from the module
    gps_data = get_current_gps()
    imu_data = get_current_orientation()
    error_data = get_current_errors()  # Optional: fetch error data if relevant

    # Print the fetched data
    print("GPS Data:", gps_data)
    print("IMU Data:", imu_data)
    if error_data:  # Check if there's any error data to display
        print("Error Data:", error_data)

    sleep(2)  # Adjust the frequency as needed based on your system's requirements
