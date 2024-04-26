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
    radius = 6371000  # Earth radius in meters

    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return (radius * c) / 100

def calculate_bearing(pointA, pointB):
    """Calculate the bearing between two points."""
    lat1, lon1 = map(math.radians, pointA)
    lat2, lon2 = map(math.radians, pointB)
    dlon = lon2 - lon1
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    bearing = math.atan2(x, y)
    return (math.degrees(bearing) + 360) % 360

def navigate_to_goal(start_gps, goal_gps, robot):
    """Navigate from start to goal using simple straight-line approximation."""
    distance = haversine(start_gps, goal_gps)
    bearing = calculate_bearing(start_gps, goal_gps)

    # Simulate navigation: For testing, let's assume one command per decision point
    print(f"Distance: {distance} cm, Bearing: {bearing} degrees")
    
    # Adjust robot's bearing to match the goal bearing
    current_yaw = get_current_heading()
    turn_needed = bearing - current_yaw
    if turn_needed > 0:
        robot.turn_right(abs(turn_needed))
    else:
        robot.turn_left(abs(turn_needed))
    
    # For simplicity, let's assume we move in steps of 10 cm
    steps = int(distance / 10)
    for _ in range(steps):
        robot.forward(30)  # Adjust power as necessary for real robot speed

    robot.stop()
    print("Arrived at the destination")

# Example usage:
if __name__ == "__main__":
    robot = Robot(config)
    current_lat, current_lon = get_current_gps()  # Make sure this function returns the latest valid GPS coordinates
    start_gps = (current_lat, current_lon)
    navigate_to_goal(start_gps, goal_gps, robot)
