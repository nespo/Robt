import math
import time
import sys
import threading
import signal
import os

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
        self.Kp = 0.5
        self.Ki = 0.1
        self.Kd = 0.05
        self.integral = 0
        self.previous_error = 0
        self.running = True
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        print(f"Initialized AutonomousPiCar with target coordinates: ({self.target_lat}, {self.target_lon})")

        # Initial GPS data retrieval
        self.initialize_gps_data()

        if self.running:
            self.navigate_obstacles_thread = threading.Thread(target=self.navigate_obstacles)
            self.navigate_obstacles_thread.start()
            self.navigation_thread = threading.Thread(target=self.navigate_to_target)
            self.navigation_thread.start()

    def initialize_gps_data(self):
        max_attempts = 100
        attempt = 0
        while attempt < max_attempts:
            initial_data = self.get_valid_gps_data()
            if initial_data:
                self.previous_lat, self.previous_lon, _ = initial_data
                print(f"Initial GPS coordinates set: ({self.previous_lat}, {self.previous_lon})")
                return
            else:
                attempt += 1
                print(f"Attempt {attempt} failed to get valid GPS data. Retrying...")
                time.sleep(2)

        self.running = False
        print("Failed to get initial GPS data after multiple attempts.")

    def navigate_to_target(self):
        while not self.stop_event.is_set():
            gps_data = self.get_valid_gps_data()
            if gps_data is None:
                print("No valid GPS data available. Waiting...")
                time.sleep(2)
                continue

            current_lat, current_lon = gps_data
            self.perform_navigation(current_lat, current_lon)

    def get_valid_gps_data(self):
        data = get_current_gps()
        if data and -90 <= data[0] <= 90 and -180 <= data[1] <= 180:
            return data
        else:
            print("Invalid or no GPS data received.")
            return None

    def perform_navigation(self, current_lat, current_lon):
        try:
            print("Starting navigation to target...")
            self.total_distance = self.calculate_distance((self.previous_lat, self.previous_lon), (self.target_lat, self.target_lon))
            print(f"Initial GPS coordinates: ({self.previous_lat}, {self.previous_lon})")
            print(f"Total distance to target: {self.total_distance:.2f} meters")

            while not self.stop_event.is_set():
                print(f"Current GPS coordinates: ({current_lat}, {current_lon})")
                current_heading = get_current_heading()
                target_heading = self.calculate_heading_difference(current_lat, current_lon)

                print(f"Current heading: {current_heading} degrees")
                print(f"Calculated target heading: {target_heading} degrees")

                self.adjust_heading(current_heading, target_heading)
                proximity = self.calculate_proximity(current_lat, current_lon)
                motor_power = self.calculate_motor_power(proximity)
                print(f"Proximity to target: {proximity:.4f} degrees")
                print(f"Setting motor power to: {motor_power}")

                with self.lock:
                    self.robot.forward(motor_power)

                self.update_distance(current_lat, current_lon, self.previous_lat, self.previous_lon)
                if self.is_target_reached(current_lat, current_lon):
                    print("Target reached.")
                    break
                self.previous_lat, self.previous_lon = current_lat, current_lon
                time.sleep(1)
        except Exception as e:
            print(f"An error occurred during navigation: {e}")

    def navigate_obstacles(self):
        try:
            while not self.stop_event.is_set():
                scan_results = []
                for angle in range(-90, 91, 18):
                    status = self.us_sensor.get_status_at(angle)
                    scan_results.append(status)

                if all(status == 2 for status in scan_results):
                    with self.lock:
                        self.robot.forward(70)
                elif any(status == 0 for status in scan_results):
                    with self.lock:
                        self.robot.backward(50)
                    time.sleep(1)
                    if scan_results.index(0) < len(scan_results) / 2:
                        with self.lock:
                            self.robot.turn_right(70)
                    else:
                        with self.lock:
                            self.robot.turn_left(70)
                    time.sleep(1)
                else:
                    with self.lock:
                        self.robot.forward(50)
                time.sleep(0.1)
        except Exception as e:
            print(f"An error occurred while navigating obstacles: {e}")

    def calculate_proximity(self, current_lat, current_lon):
        return math.hypot(current_lat-self.target_lat, current_lon-self.target_lon)

    def update_distance(self, current_lat, current_lon, previous_lat, previous_lon):
        distance_traveled = self.calculate_distance((previous_lat, previous_lon), (current_lat, current_lon))
        self.total_distance = max(0, self.total_distance - distance_traveled)
        print(f"Traveled {distance_traveled:.2f} m, Remaining distance to target: {self.total_distance:.2f} m")

    def calculate_motor_power(self, proximity):
        return max(20, 50 - int(proximity * 1000))

    def adjust_heading(self, current_heading, target_heading):
        error = self.calculate_heading_difference(current_heading, target_heading)
        turn_power = int(self.Kp * error + self.Ki * self.integral + self.Kd * (error - self.previous_error))
        self.previous_error = error
        self.integral += error
        turn_power = max(-50, min(turn_power, 50))
        print(f"Adjusting heading by {turn_power} degrees")
        with self.lock:
            if turn_power > 0:
                self.robot.turn_right(abs(turn_power))
            else:
                self.robot.turn_left(abs(turn_power))
        time.sleep(0.5)

    def calculate_distance(self, point1, point2):
        lat1, lon1 = point1
        lat2, lon2 = point2
        radius = 6371000
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = math.sin(dlat/2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon/2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return radius * c

    def calculate_heading_difference(self, current_heading, target_heading):
        difference = target_heading - current_heading
        if difference > 180:
            difference -= 360
        elif difference < -180:
            difference += 360
        return difference

    def is_target_reached(self, current_lat, current_lon):
        return self.calculate_proximity(current_lat, current_lon) < 0.0001

    def stop(self):
        self.stop_event.set()
        with self.lock:
            self.robot.stop()
        if self.navigate_obstacles_thread.is_alive():
            self.navigate_obstacles_thread.join()
        if self.navigation_thread.is_alive():
            self.navigation_thread.join()
        print("Robot completely stopped.")

def setup_signal_handlers(car):
    def handle_signal(signum, frame):
        print(f"Signal {signum} received, stopping car...")
        car.stop()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

# Example Usage
if __name__ == "__main__":
    target_latitude = 62.878800
    target_longitude = 27.637387
    us_sensor = Ultrasonic(Pin('D8'), Pin('D9'))
    robot = Robot(config)
    autonomous_car = AutonomousPiCar(target_latitude, target_longitude, robot, us_sensor)
    setup_signal_handlers(autonomous_car)
