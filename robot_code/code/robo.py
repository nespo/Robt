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
from robot_code.code.config import config
from robot_code.modules.nav import get_current_gps, get_current_heading


class AutonomousPiCar:
    def __init__(self, target_lat, target_lon, robot):
        self.target_lat = target_lat
        self.target_lon = target_lon
        self.robot = robot

    def navigate_to_target(self):
        while not self.is_target_reached():
            current_lat, current_lon = get_current_gps()
            if self.is_target_reached(current_lat, current_lon):
                print("Target reached!")
                self.stop()  # Ensure the robot stops when it reaches the destination
                break

            current_heading = get_current_heading()
            target_heading = self.calculate_heading_to_target(current_lat, current_lon)

            self.adjust_heading(current_heading, target_heading)
            # Use a lower power setting for more precise movements as you get closer to the target
            proximity = self.calculate_proximity(current_lat, current_lon)
            motor_power = max(20, 50 - int(proximity * 1000))  # Decrease power as we get closer
            self.robot.forward(motor_power)
            time.sleep(1)  # Short pause to allow for real-time updates

    def adjust_heading(self, current_heading, target_heading):
        heading_difference = self.calculate_heading_difference(current_heading, target_heading)
        if abs(heading_difference) > 20:  # More precise turning threshold
            turn_power = min(50, max(20, abs(heading_difference) * 5))  # Adjust power based on the angle needed
            if heading_difference > 0:
                self.robot.turn_right(turn_power)
            else:
                self.robot.turn_left(turn_power)
            time.sleep(0.5)  # Adjust duration based on your robot's turning speed

    def calculate_heading_difference(self, current_heading, target_heading):
        difference = target_heading - current_heading
        if difference > 180:
            difference -= 360
        elif difference < -180:
            difference += 360
        return difference

    def calculate_heading_to_target(self, current_lat, current_lon):
        # Calculates the bearing to the target using the haversine formula
        lat1 = math.radians(current_lat)
        lat2 = math.radians(self.target_lat)
        diffLong = math.radians(self.target_lon - current_lon)
        x = math.sin(diffLong) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(diffLong))
        initial_bearing = math.atan2(x, y)
        initial_bearing = math.degrees(initial_bearing)
        compass_bearing = (initial_bearing + 360) % 360
        return compass_bearing

    def calculate_proximity(self, current_lat, current_lon):
        # Calculate the Euclidean distance to the target for power adjustment
        return math.hypot(current_lat-self.target_lat, current_lon-self.target_lon)

    def is_target_reached(self, current_lat, current_lon):
        # More precise threshold for reaching the target, e.g., 5 meters approximated by degrees
        return self.calculate_proximity(current_lat, current_lon) < 0.00005

    def stop(self):
        # Command to stop all motors
        self.robot.stop(0)

# Example Usage
target_latitude = 62.880338  # Los Angeles latitude 62.880338, 27.635195
target_longitude = 27.635195  # Los Angeles longitude
robot = AutonomousPiCar(target_latitude, target_longitude, Robot(config))
robot.navigate_to_target()
