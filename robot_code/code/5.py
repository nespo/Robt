import math
import time


import os
import sys

# Correct library import paths
script_dir = os.path.dirname(os.path.realpath(__file__))
parent_dir = os.path.join(script_dir, '..', '..')
sys.path.append(os.path.abspath(parent_dir))

# Import robot-specific modules
from robot_code.code.motor_control import Robot
from robot_code.modules.ultrasonic import Ultrasonic
from robot_code.modules.pin import Pin
from robot_code.code.config import config
from robot_code.modules.nav import get_current_gps, get_current_heading

robot = Robot(config)
# GPS and orientation functions
def haversine_distance(lat1, lon1, lat2, lon2):
    R = 6371000  # Earth radius in meters
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return (R * c)

def calculate_bearing(lat1, lon1, lat2, lon2):
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_lambda = math.radians(lon2 - lon1)
    x = math.sin(delta_lambda) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)
    bearing = math.atan2(x, y)
    return math.degrees(bearing) % 360

def navigate_to_waypoint(current_yaw, target_bearing, distance, turn_power_reduction):
    # Determine turn direction
    angle_diff = (target_bearing - current_yaw + 360) % 360
    if angle_diff > 180:
        robot.turn_left(100, turn_power_reduction)  # Full power turn
    else:
        robot.turn_right(100, turn_power_reduction)

    # Move forward based on distance
    if distance > 10:
        robot.forward(100)  # Full power forward
    else:
        robot.forward(int(distance * 10))  # Reduce power as it gets closer to the target

# Obstacle detection and avoidance
def scan_for_obstacles(ultrasonic_sensor, reference_distance=35):
    # Scan using the Ultrasonic class and its scanning step method
    return ultrasonic_sensor.scan_step(reference_distance)

def avoid_obstacles(obstacle_data, turn_power_reduction):
    # Avoidance strategy based on the obstacle data
    for status in obstacle_data:
        if status == 0:  # 'Danger/close'
            robot.turn_right(100, turn_power_reduction)
            break

# Main control loop
def main_control_loop(target_lat, target_lon, ultrasonic_sensor, turn_power_reduction):
    try:
        while True:
            current_lat, current_lon = get_current_gps()  # Replace with actual GPS reading
            current_yaw = get_current_heading()  # Replace with actual compass reading
            target_bearing = calculate_bearing(current_lat, current_lon, target_lat, target_lon)
            distance = haversine_distance(current_lat, current_lon, target_lat, target_lon)

            print(current_lat, current_lon, current_yaw, target_bearing, distance)
            if distance < 0.5:  # When target is closer than 1 meter, it's considered reached
                print("Target reached!")
                break

            obstacle_data = scan_for_obstacles(ultrasonic_sensor)
            if obstacle_data:  # Non-empty data implies obstacles detected
                avoid_obstacles(obstacle_data, turn_power_reduction)
            else:
                navigate_to_waypoint(current_yaw, target_bearing, distance, turn_power_reduction)

            time.sleep(1)
    except KeyboardInterrupt:
        print("Program stopped manually.")

if __name__ == '__main__':
    ultrasonic_sensor = Ultrasonic(Pin("D8"), Pin("D9"))  # Initialize with actual trigger and echo pins
    target_latitude = 62.878815 # Example target latitude
    target_longitude = 27.637536  # Example target longitude
    turn_power_reduction = 0.7
    main_control_loop(target_latitude, target_longitude, ultrasonic_sensor, turn_power_reduction)