import math
import time
import os
import sys
import threading

# Correct library import paths
script_dir = os.path.dirname(os.path.realpath(__file__))
parent_dir = os.path.join(script_dir, '..', '..')
sys.path.append(os.path.abspath(parent_dir))

from robot_code.code.motor_control import Robot
from robot_code.code.config import config
from robot_code.modules.nav import get_current_gps, get_current_heading
from robot_code.modules.ultrasonic import Ultrasonic  
from robot_code.modules.pin import Pin

class AutonomousPiCar:
    def __init__(self, target_lat, target_lon, robot, us_sensor):
        self.target_lat = target_lat
        self.target_lon = target_lon
        self.robot = robot
        self.us_sensor = us_sensor
        self.travel_path = []
        self.Kp = 0.5  # Proportional gain for PID controller
        self.Ki = 0.1  # Integral gain for PID controller
        self.Kd = 0.05  # Derivative gain for PID controller
        self.previous_error = 0
        self.integral = 0
        self.total_distance = 0
        self.running = True  # Flag to control thread operation
        self.obstacle_thread = threading.Thread(target=self.navigate_obstacles)
        self.lock = threading.Lock()  # Lock for thread-safe operations on the robot

        print(f"Initialized AutonomousPiCar with target coordinates: ({self.target_lat}, {self.target_lon})")
        self.obstacle_thread.start()

    def navigate_to_target(self):
        print("Starting navigation to target...")
        previous_lat, previous_lon, previous_quality = self.get_valid_gps_data()

        self.travel_path.append((previous_lat, previous_lon))
        print(f"Initial GPS coordinates: ({previous_lat}, {previous_lon})")
        self.total_distance = self.calculate_total_distance_to_target(previous_lat, previous_lon)
        print(f"Total distance to target: {self.total_distance:.2f} meters")

        try:
            while not self.is_target_reached(previous_lat, previous_lon) and self.running:
                current_lat, current_lon, _ = self.get_valid_gps_data()
                self.travel_path.append((current_lat, current_lon))
                print(f"Current GPS coordinates: ({current_lat}, {current_lon})")

                current_heading = get_current_heading()
                print(f"Current heading: {current_heading} degrees")
                target_heading = self.calculate_heading_to_target(current_lat, current_lon)
                print(f"Calculated target heading: {target_heading} degrees")

                self.adjust_heading(current_heading, target_heading)
                proximity = self.calculate_proximity(current_lat, current_lon)
                print(f"Proximity to target: {proximity:.4f} degrees")
                motor_power = self.calculate_motor_power(proximity)
                print(f"Setting motor power to: {motor_power}")
                
                with self.lock:
                    self.robot.forward(motor_power)

                time.sleep(1)

                self.update_distance(current_lat, current_lon, previous_lat, previous_lon)
                previous_lat, previous_lon = current_lat, current_lon
        except KeyboardInterrupt:
            print("KeyboardInterrupt caught. Stopping robot...")
            self.stop()

    def get_valid_gps_data(self):
        """Fetch GPS data, ensuring it meets validity criteria including HDOP or fix quality."""
        lat, lon, quality = get_current_gps()  # Assuming get_current_gps now also returns 'quality'
        while lat is None or lon is None or not self.is_valid_gps_data(lat, lon) or quality < 2:
            time.sleep(1)  # Wait and retry if the data is not valid
            lat, lon, quality = get_current_gps()
        return lat, lon, quality

    def is_valid_gps_data(self, lat, lon):
        """Validate GPS data within the standard range for latitude and longitude."""
        return -90 <= lat <= 90 and -180 <= lon <= 180

    def navigate_obstacles(self):
        while self.running:
            scan_results = []
            for angle in range(-90, 91, 18):
                status = self.us_sensor.get_status_at(angle)
                scan_results.append(status)

            with self.lock:
                if all(status == 2 for status in scan_results):
                    self.robot.forward(70)
                elif any(status == 0 for status in scan_results):
                    self.robot.backward(50)
                    time.sleep(1)
                    if scan_results.index(0) < len(scan_results) / 2:
                        self.robot.turn_right(70)
                    else:
                        self.robot.turn_left(70)
                    time.sleep(1)
                else:
                    self.robot.forward(50)
            time.sleep(0.1)

    def update_distance(self, current_lat, current_lon, previous_lat, previous_lon):
        distance_traveled = self.calculate_distance((previous_lat, previous_lon), (current_lat, current_lon))
        self.total_distance -= distance_traveled
        print(f"Traveled {distance_traveled:.2f} m, Remaining distance to target: {self.total_distance:.2f} m")

    def calculate_motor_power(self, proximity):
        return max(20, 50 - int(proximity * 1000))

    def adjust_heading(self, current_heading, target_heading):
        error = self.calculate_heading_difference(current_heading, target_heading)
        self.integral += error
        derivative = error - self.previous_error
        turn_power = int(self.Kp * error + self.Ki * self.integral + self.Kd * derivative)
        self.previous_error = error

        if abs(turn_power) > 50:
            turn_power = 50 * (1 if turn_power > 0 else -1)

        print(f"Adjusting heading by {turn_power} degrees")
        if turn_power > 0:
            self.robot.turn_right(abs(turn_power))
        else:
            self.robot.turn_left(abs(turn_power))
        time.sleep(0.5)

    def calculate_total_distance_to_target(self, current_lat, current_lon):
        return self.calculate_distance((current_lat, current_lon), (self.target_lat, self.target_lon))

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
        self.running = False
        self.robot.stop()
        self.obstacle_thread.join()  # Wait for the obstacle thread to finish
        print("Robot stopped.")

# Example Usage
target_latitude = 62.878800
target_longitude = 27.637387
us_sensor = Ultrasonic(Pin('D8'), Pin('D9'))  # Assuming this is the correct way to initialize
robot = Robot(config)
autonomous_car = AutonomousPiCar(target_latitude, target_longitude, robot, us_sensor)
autonomous_car.navigate_to_target()