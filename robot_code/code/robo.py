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
        self.travel_path = []  # To store the path traveled
        print(f"Initialized AutonomousPiCar with target coordinates: ({self.target_lat}, {self.target_lon})")

    def navigate_to_target(self):
        try:
            print("Starting navigation to target...")
            current_lat, current_lon = get_current_gps()
            self.travel_path.append((current_lat, current_lon))  # Store initial position
            print(f"Initial GPS coordinates: ({current_lat}, {current_lon})")
            while not self.is_target_reached(current_lat, current_lon):
                current_lat, current_lon = get_current_gps()
                self.travel_path.append((current_lat, current_lon))  # Append current position to path
                print(f"Current GPS coordinates: ({current_lat}, {current_lon})")
                if self.is_target_reached(current_lat, current_lon):
                    print("Target reached!")
                    self.print_total_distance()  # Print the total distance traveled
                    self.stop()
                    break

                current_heading = get_current_heading()
                print(f"Current heading: {current_heading} degrees")
                target_heading = self.calculate_heading_to_target(current_lat, current_lon)
                print(f"Calculated target heading: {target_heading} degrees")

                self.adjust_heading(current_heading, target_heading)
                proximity = self.calculate_proximity(current_lat, current_lon)
                print(f"Proximity to target: {proximity} degrees")
                motor_power = max(20, 50 - int(proximity * 1000))
                print(f"Setting motor power to: {motor_power}")
                self.robot.forward(motor_power)
                time.sleep(1)
        except KeyboardInterrupt:
            print("KeyboardInterrupt caught. Stopping robot...")
            self.stop()

    def print_total_distance(self):
        total_distance = 0
        for i in range(1, len(self.travel_path)):
            total_distance += self.calculate_distance(self.travel_path[i-1], self.travel_path[i])
        print(f"Total distance traveled: {total_distance} meters")

    def calculate_distance(self, point1, point2):
        # Uses the haversine formula to calculate distance between two points
        lat1, lon1 = point1
        lat2, lon2 = point2
        radius = 6371000  # Earth radius in meters

        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = (math.sin(dlat/2) * math.sin(dlat/2) +
             math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
             math.sin(dlon/2) * math.sin(dlon/2))
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = radius * c
        return distance
    def adjust_heading(self, current_heading, target_heading):
        heading_difference = self.calculate_heading_difference(current_heading, target_heading)
        print(f"Heading difference: {heading_difference} degrees")
        if abs(heading_difference) > 20:  # More precise turning threshold
            turn_power = min(50, max(20, abs(heading_difference) * 5))  # Adjust power based on the angle needed
            print(f"Turning {'right' if heading_difference > 0 else 'left'} with power: {turn_power}")
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
        self.robot.stop()
        print("Robot stopped.")

# Example Usage
target_latitude = 62.878800  # Los Angeles latitude 62.880338, 27.635195
target_longitude = 27.637387  # Los Angeles longitude
robot = AutonomousPiCar(target_latitude, target_longitude, Robot(config))
robot.navigate_to_target()
