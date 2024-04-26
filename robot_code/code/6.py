import time
import math
import threading

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

import heapq

# Example usage of these GPS coordinates
start_gps = (0, 0)  # This will be set by the robot's current GPS location
goal_gps = (62.878815, 27.637536)

def haversine(coord1, coord2):
    """Calculate the great-circle distance between two points on the Earth."""
    lat1, lon1 = coord1
    lat2, lon2 = coord2

    # Convert decimal degrees to radians
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)

    # Earth radius in kilometers
    R = 6371

    # Calculate the difference in latitude and longitude
    dlon = lon2 - lon1
    dlat = lat2 - lat1

    # Haversine formula
    a = math.sin(dlat / 2) * math.sin(dlat / 2) + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) * math.sin(dlon / 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    # Distance in kilometers
    distance = R * c

    return distance * 100000

def calculate_bearing(coord1, coord2):
    """
    This function calculates the initial bearing angle between two points on Earth in degrees.

    Args:
        lat1 (float): Latitude of the first point in degrees.
        lon1 (float): Longitude of the first point in degrees.
        lat2 (float): Latitude of the second point in degrees.
        lon2 (float): Longitude of the second point in degrees.

    Returns:
        float: The initial bearing angle in degrees from the first point to the second point.
    """

    lat1, lon1 = coord1
    lat2, lon2 = coord2
    # Convert decimal degrees to radians
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)

    y = math.cos(lat2) * math.sin(lon2 - lon1)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(lon2 - lon1)
    bearing = math.atan2(y, x)

    # Convert radians to degrees and adjust for quadrant
    bearing = math.degrees(bearing)
    bearing = (bearing + 360) % 360

    return bearing

def navigate_obstacles(robot, us):
    scan_results = []
    for angle in range(-90, 91, 18):
        status = us.get_status_at(angle)
        scan_results.append(status)

    # Automatically move forward but adjust based on obstacles
    if all(status == 2 for status in scan_results):
        robot.forward(70)
    elif any(status == 0 for status in scan_results):
        robot.backward(50)
        time.sleep(1)
        if scan_results.index(0) < len(scan_results) / 2:
            robot.turn_right(70)
        else:
            robot.turn_left(70)
        time.sleep(1)
    else:
        return True
    # No need to stop the robot here if we want continuous movement in auto mode

def navigate_to_goal(robot, start_gps, goal_gps):
    current_position = start_gps
    while True:
        distance = haversine(current_position, goal_gps)
        if distance < 10:
            print("Arrived at the destination")
            robot.stop()
            break

        bearing = calculate_bearing(current_position, goal_gps)
        current_yaw = get_current_heading()

        turn_needed = (bearing - current_yaw + 360) % 360
        if turn_needed > 180:
            turn_needed -= 360

        print(f"Distance: {distance} cm, Bearing: {bearing} degrees, Turn_needed: {turn_needed}")
        adjust_heading(robot, turn_needed)

        current_position = get_current_gps()

def adjust_heading(robot, turn_needed):
    if turn_needed > 5:
        robot.turn_right(min(abs(turn_needed), 30))
    elif turn_needed < -5:
        robot.turn_left(min(abs(turn_needed), 30))
    time.sleep(0.5)

def obstacle_avoidance(robot, us):
    while True:
        scan_results = [us.get_status_at(angle) for angle in range(-90, 91, 18)]
        if any(status == 0 for status in scan_results):
            robot.stop()
            handle_obstacles(robot, scan_results)

def handle_obstacles(robot, scan_results):
    obstacle_index = scan_results.index(0)
    if obstacle_index < len(scan_results) / 2:
        robot.turn_right(90)
    else:
        robot.turn_left(90)
    robot.forward(100)
    time.sleep(1)

def main():
    robot = Robot(config)
    us = Ultrasonic(Pin('D8'), Pin('D9'))
    current_lat, current_lon = get_current_gps()
    start_gps = (current_lat, current_lon)
    goal_gps = (62.878815, 27.637536)

    # Start threads
    navigation_thread = threading.Thread(target=navigate_to_goal, args=(robot, start_gps, goal_gps))
    obstacle_thread = threading.Thread(target=obstacle_avoidance, args=(robot, us))

    navigation_thread.start()
    obstacle_thread.start()

    navigation_thread.join()
    obstacle_thread.join()

if __name__ == "__main__":
    main()