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

def navigate_to_goal(start_gps, goal_gps, robot):
    """Navigate from start to goal, dynamically adjusting the path."""
    current_position = start_gps

    while True:
        distance = haversine(current_position, goal_gps)
        if distance < 10:  # If within 10 cm of the goal, consider it reached
            print("Arrived at the destination")
            robot.stop()
            break

        bearing = calculate_bearing(current_position, goal_gps)
        current_yaw = get_current_heading()

        turn_needed = bearing - current_yaw

        print(f"Distance: {distance} cm, Bearing: {bearing} degrees, Turn_needed: {turn_needed}")

        # Adjust the robot's bearing to match the goal bearing
        if turn_needed > 0:
            robot.turn_right(abs(turn_needed))
            time.sleep(1)
        else:
            robot.turn_left(abs(turn_needed))
            time.sleep(1)
        
        # Move forward in small increments to continuously adjust the path
        robot.forward(50)  # Adjust power as necessary for real robot speed
        time.sleep(1)  # Assuming a delay to allow for movement before the next GPS read

        # Update current_position with the new GPS coordinates
        current_position = get_current_gps()

        print(f"Moving to position: {current_position}, Distance left: {distance} cm, Bearing needed: {bearing} degrees")

# Example usage:
if __name__ == "__main__":
    robot = Robot(config)
    current_lat, current_lon = get_current_gps()  # Make sure this function returns the latest valid GPS coordinates
    start_gps = (current_lat, current_lon)
    navigate_to_goal(start_gps, goal_gps, robot)
