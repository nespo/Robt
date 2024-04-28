import math
import time
import os
import sys
import threading
import signal

# Adjust library import paths as necessary
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
        self.Kp = 0.5
        self.Ki = 0.1
        self.Kd = 0.05
        self.previous_error = 0
        self.integral = 0
        self.total_distance = 0
        self.running = True
        self.lock = threading.Lock()
        self.initialize_signals()
        print(f"Initialized AutonomousPiCar with target coordinates: ({self.target_lat}, {self.target_lon})")
        self.navigate_obstacles_thread = threading.Thread(target=self.navigate_obstacles)
        self.navigate_obstacles_thread.start()

    def initialize_signals(self):
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)  # Handle SIGTERM as well

    def signal_handler(self, signum, frame):
        print("Signal interrupt caught, stopping all operations...")
        self.stop()

    def navigate_to_target(self):
        print("Starting navigation to target...")
        current_lat, current_lon, _ = self.get_valid_gps_data()
        if current_lat is None:
            print("No valid GPS data available. Navigation halted.")
            return

        self.total_distance = self.calculate_total_distance_to_target(current_lat, current_lon)
        print(f"Initial GPS coordinates: ({current_lat}, {current_lon})")
        print(f"Total distance to target: {self.total_distance:.2f} meters")

        while self.running:
            current_lat, current_lon, _ = self.get_valid_gps_data()
            if current_lat is None:
                print("Waiting for valid GPS data...")
                time.sleep(2)
                continue

            print(f"Current GPS coordinates: ({current_lat}, {current_lon})")
            current_heading = get_current_heading()
            target_heading = self.calculate_heading_to_target(current_lat, current_lon)

            print(f"Current heading: {current_heading} degrees")
            print(f"Calculated target heading: {target_heading} degrees")

            self.adjust_heading(current_heading, target_heading)
            proximity = self.calculate_proximity(current_lat, current_lon)
            motor_power = self.calculate_motor_power(proximity)
            print(f"Proximity to target: {proximity:.4f} degrees")
            print(f"Setting motor power to: {motor_power}")

            with self.lock:
                self.robot.forward(motor_power)

            self.update_distance(current_lat, current_lon, previous_lat, previous_lon)
            if self.is_target_reached(current_lat, current_lon):
                print("Target reached.")
                break
            previous_lat, previous_lon = current_lat, current_lon
            time.sleep(1)

    def get_valid_gps_data(self):
        data = get_current_gps()
        if data and -90 <= data[0] <= 90 and -180 <= data[1] <= 180:
            return data[0], data[1], data[2] if len(data) > 2 else 1
        else:
            print("Invalid or no GPS data received. Retrying...")
            time.sleep(1)  # Retry interval
            return self.get_valid_gps_data()  # Recursion to ensure valid data

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
        self.total_distance = max(0, self.total_distance - distance_traveled)
        print(f"Traveled {distance_traveled:.2f} m, Remaining distance to target: {self.total_distance:.2f} m")

    def calculate_motor_power(self, proximity):
        # Adjust motor power based on proximity to target
        return max(20, 50 - int(proximity * 1000))

    def adjust_heading(self, current_heading, target_heading):
        error = self.calculate_heading_difference(current_heading, target_heading)
        turn_power = int(self.Kp * error + self.Ki * self.integral + self.Kd * (error - self.previous_error))
        self.previous_error = error
        self.integral += error
        turn_power = max(-50, min(turn_power, 50))  # Clamp to [-50, 50]
        print(f"Adjusting heading by {turn_power} degrees")
        with self.lock:
            if turn_power > 0:
                self.robot.turn_right(abs(turn_power))
            else:
                self.robot.turn_left(abs(turn_power))
        time.sleep(0.5)

    def calculate_distance(self, point1, point2):
        # Haversine formula to calculate distance
        lat1, lon1 = point1
        lat2, lon2 = point2
        radius = 6371000  # Earth radius in meters
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = math.sin(dlat/2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon/2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return radius * c

    def calculate_heading_difference(self, current_heading, target_heading):
        # Calculate the shortest path between current and target headings
        difference = target_heading - current_heading
        if difference > 180:
            difference -= 360
        elif difference < -180:
            difference += 360
        return difference

    def is_target_reached(self, current_lat, current_lon):
        # Check if the target is reached within a small threshold
        return self.calculate_proximity(current_lat, current_lon) < 0.0001  # Adjust threshold as needed

    def stop(self):
        self.running = False
        with self.lock:
            self.robot.stop()
        self.navigate_obstacles_thread.join()
        print("Robot completely stopped.")

# Example Usage
target_latitude = 62.878800
target_longitude = 27.637387
us_sensor = Ultrasonic(Pin('D8'), Pin('D9'))  # Example sensor pins
robot = Robot(config)
autonomous_car = AutonomousPiCar(target_latitude, target_longitude, robot, us_sensor)
autonomous_car.navigate_to_target()
