import math
import time
import sys
import threading
import signal
import os
import logging
import numpy as np
from functools import wraps

# Assume you have YOLOv5 for camera-based object detection
from yolov5 import YOLOv5

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Correct library import paths
script_dir = os.path.dirname(os.path.realpath(__file__))
parent_dir = os.path.join(script_dir, '..', '..')
sys.path.append(os.path.abspath(parent_dir))

from robot_code.code.motor_control import Robot
from robot_code.code.config import config
from robot_code.modules.nav import get_current_gps, get_current_heading
from robot_code.modules.ultrasonic import Ultrasonic
from robot_code.modules.pin import Pin

class KalmanFilter:
    def __init__(self, process_variance, measurement_variance, estimated_error):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimated_error = estimated_error
        self.current_estimate = 0
        self.last_estimate = 0
        self.kalman_gain = 0

    def update(self, measurement):
        self.kalman_gain = self.estimated_error / (self.estimated_error + self.measurement_variance)
        self.current_estimate = self.last_estimate + self.kalman_gain * (measurement - self.last_estimate)
        self.estimated_error = (1 - self.kalman_gain) * self.estimated_error + self.process_variance
        self.last_estimate = self.current_estimate
        return self.current_estimate

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
        self.lat_kf = KalmanFilter(0.01, 0.03, 1.0)
        self.lon_kf = KalmanFilter(0.01, 0.03, 1.0)
        logging.info(f"Initialized AutonomousPiCar with target coordinates: ({self.target_lat}, {self.target_lon})")

        self.initialize_gps_data()

        if self.running:
            self.navigate_obstacles_thread = threading.Thread(target=self.navigate_obstacles)
            self.navigate_obstacles_thread.start()
            self.navigation_thread = threading.Thread(target=self.navigate_to_target)
            self.navigation_thread.start()

    def retry(attempts):
        def retry_decorator(func):
            @wraps(func)
            def wrapper(*args, **kwargs):
                last_exception = None
                for _ in range(attempts):
                    try:
                        return func(*args, **kwargs)
                    except Exception as e:
                        last_exception = e
                        time.sleep(2)
                raise last_exception
            return wrapper
        return retry_decorator

    @retry(attempts=100)
    def initialize_gps_data(self):
        initial_data = self.get_valid_gps_data()
        if initial_data:
            self.previous_lat, self.previous_lon = initial_data
            logging.info(f"Initial GPS coordinates set: ({self.previous_lat}, {self.previous_lon})")
            return
        raise RuntimeError("Failed to get initial GPS data after multiple attempts.")

    def navigate_to_target(self):
        while not self.stop_event.is_set():
            gps_data = self.get_valid_gps_data()
            if gps_data is None:
                logging.info("No valid GPS data available. Waiting...")
                time.sleep(2)
                continue

            current_lat, current_lon = gps_data
            self.perform_navigation(current_lat, current_lon)

    def get_valid_gps_data(self):
        data = get_current_gps()
        if data and -90 <= data[0] <= 90 and -180 <= data[1] <= 180:
            smooth_lat = self.lat_kf.update(data[0])
            smooth_lon = self.lon_kf.update(data[1])
            return (smooth_lat, smooth_lon)
        else:
            logging.warning("Invalid or no GPS data received.")
            return None

    def perform_navigation(self, current_lat, current_lon):
        try:
            logging.info("Starting navigation to target...")
            self.total_distance = self.calculate_distance((self.previous_lat, self.previous_lon), (self.target_lat, self.target_lon))
            logging.info(f"Initial GPS coordinates: ({self.previous_lat}, {self.previous_lon})")
            logging.info(f"Total distance to target: {self.total_distance:.2f} meters")

            while not self.stop_event.is_set():
                logging.info(f"Current GPS coordinates: ({current_lat}, {current_lon})")
                current_heading = get_current_heading()
                target_heading = self.calculate_heading_difference(current_lat, current_lon)

                logging.info(f"Current heading: {current_heading} degrees")
                logging.info(f"Calculated target heading: {target_heading} degrees")

                self.adjust_heading(current_heading, target_heading)
                proximity = self.calculate_proximity(current_lat, current_lon)
                motor_power = self.calculate_motor_power(proximity)
                logging.info(f"Proximity to target: {proximity:.4f} degrees")
                logging.info(f"Setting motor power to: {motor_power}")

                with self.lock:
                    self.robot.forward(motor_power)

                self.update_distance(current_lat, current_lon, self.previous_lat, self.previous_lon)
                if self.is_target_reached(current_lat, current_lon):
                    logging.info("Target reached.")
                    break
                self.previous_lat, self.previous_lon = current_lat, current_lon
                time.sleep(1)
        except Exception as e:
            logging.error(f"An error occurred during navigation: {e}")

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
            logging.error(f"An error occurred while navigating obstacles: {e}")

    def calculate_proximity(self, current_lat, current_lon):
        return math.hypot(current_lat - self.target_lat, current_lon - self.target_lon)

    def update_distance(self, current_lat, current_lon, previous_lat, previous_lon):
        distance_traveled = self.calculate_distance((previous_lat, previous_lon), (current_lat, current_lon))
        self.total_distance = max(0, self.total_distance - distance_traveled)
        logging.info(f"Traveled {distance_traveled:.2f} m, Remaining distance to target: {self.total_distance:.2f} m")

    def calculate_motor_power(self, proximity):
        return max(20, 50 - int(proximity * 1000))

    def adjust_heading(self, current_heading, target_heading):
        error = self.calculate_heading_difference(current_heading, target_heading)
        turn_power = int(self.Kp * error + self.Ki * self.integral + self.Kd * (error - self.previous_error))
        self.previous_error = error
        self.integral += error
        turn_power = max(-50, min(turn_power, 50))
        logging.info(f"Adjusting heading by {turn_power} degrees")
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
        a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
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
        logging.info("Robot completely stopped.")

def setup_signal_handlers(car):
    def handle_signal(signum, frame):
        logging.info(f"Signal {signum} received, stopping car...")
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
