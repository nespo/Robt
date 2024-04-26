import time
import math

import os, sys
# Adjust the sys.path to include the parent directory of robot_code
script_dir = os.path.dirname(__file__)
parent_dir = os.path.join(script_dir, '..', '..')
sys.path.append(os.path.abspath(parent_dir))

from config import config
from robot_code.modules.speed import Speed
from robot_code.modules.servo import Servo
from robot_code.modules.pwm import PWM
from robot_code.modules.pin import Pin
from robot_code.modules.ultrasonic import Ultrasonic
from robot_code.modules.nav import get_current_gps, get_current_heading
from robot_code.code.motor_control import Robot

# Initialization
speed_controller = Robot(config)
ultrasonic_sensor = Ultrasonic(Pin('D8'), Pin('D9'))
target_waypoints = [(62.878817, 27.637539), (62.878815, 27.637536)]  # List of waypoints (lat, lon)
current_waypoint_index = 0

def calculate_bearing(lat1, lon1, lat2, lon2):
    # This function calculates the bearing between two GPS coordinates
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_lon = math.radians(lon2 - lon1)
    x = math.sin(delta_lon) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(delta_lon)
    bearing = math.atan2(x, y)
    return math.degrees(bearing) % 360

def haversine_distance(lat1, lon1, lat2, lon2):
    # This function calculates the distance between two GPS coordinates
    R = 6371000  # Radius of Earth in meters
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    print(f"haversine_distance: {(R * c) / 100} cm")
    return (R * c) / 100

def navigate_to_waypoint(current_lat, current_lon, target_lat, target_lon):
    target_bearing = calculate_bearing(current_lat, current_lon, target_lat, target_lon)
    distance = haversine_distance(current_lat, current_lon, target_lat, target_lon)
    current_yaw = get_current_heading()
    angle_diff = (target_bearing - current_yaw + 360) % 360
    
    # Determine turn direction and magnitude
    if angle_diff > 180:
        speed_controller.turn_left(100)  # Adjust as necessary
    else:
        speed_controller.turn_right(100)
    
    # Move forward based on distance to the next waypoint
    if distance > 100:
        speed_controller.forward(80)  # Moderate speed
    else:
        speed_controller.forward(int(distance * 8))  # Slow down as it gets closer

def main_control_loop():
    global current_waypoint_index
    try:
        while current_waypoint_index < len(target_waypoints):
            current_lat, current_lon = get_current_gps()  # Fetch current GPS coordinates
            target_lat, target_lon = target_waypoints[current_waypoint_index]
            
            # Check if current waypoint is reached
            if haversine_distance(current_lat, current_lon, target_lat, target_lon) < 0.5:
                print("Waypoint reached: ", target_lat, target_lon)
                current_waypoint_index += 1  # Move to next waypoint
                if current_waypoint_index >= len(target_waypoints):
                    print("All waypoints reached.")
                    break
                continue
            
            navigate_to_waypoint(current_lat, current_lon, target_lat, target_lon)
            time.sleep(1)

            # Check for obstacles
            distance = ultrasonic_sensor.get_distance()
            if distance < 30:  # distance in cm
                speed_controller.stop()  # Stop if an obstacle is too close
                print("Obstacle detected! Stopping.")
                time.sleep(2)  # Wait for a bit before trying again

    except KeyboardInterrupt:
        print("Program stopped manually.")
    finally:
        speed_controller.stop()

if __name__ == '__main__':
    main_control_loop()
