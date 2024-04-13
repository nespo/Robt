import numpy as np
import time
import logging
from threading import Thread
from rplidar import RPLidar, RPLidarException

import os, sys
# Adjust the sys.path to include the parent directory of robot_code
script_dir = os.path.dirname(__file__)
parent_dir = os.path.join(script_dir, '..', '..')
sys.path.append(os.path.abspath(parent_dir))

from robot_code.modules.pin import Pin
from robot_code.modules.ultrasonic import Ultrasonic

# Configure logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

class LidarScanner:
    def __init__(self, port):
        self.lidar = RPLidar(port)
        try:
            self.lidar.connect()
            logging.info("LIDAR connected.")
        except RPLidarException as e:
            logging.error(f"Failed to connect to LIDAR: {e}")
            raise SystemExit(e)

    def iter_scans(self):
        try:
            for scan in self.lidar.iter_scans():
                scan_data = {round(measurement[1]): measurement[2] for measurement in scan if measurement[0] > 0}
                yield scan_data  # Use yield instead of return
        except RPLidarException as e:
            logging.error(f"Lidar scanning error: {e}")
            self.close()
            raise


    def close(self):
        self.lidar.stop()
        self.lidar.disconnect()
        logging.info("LIDAR disconnected safely.")

class ObstacleChecker:
    def __init__(self, lidar, us, config):
        self.lidar = lidar
        self.us = us
        self.config = config
        self.us_thread = None
        self.lidar_data = {}
        self.us_data = {}

    def get_lidar_data(self):
        self.lidar_data = self.lidar.iter_scans()

    def full_ultrasonic_sweep(self):
        self.us_data = {}
        for angle in range(-self.us.ANGLE_RANGE // 2, self.us.ANGLE_RANGE // 2 + 1, self.us.STEP):
            self.us_data[angle] = self.us.get_distance_at(angle)
            time.sleep(self.us.SERVO_SET_DELAY)

    def start_ultrasonic_sweep(self):
        if self.us_thread is None or not self.us_thread.is_alive():
            self.us_thread = Thread(target=self.full_ultrasonic_sweep)
            self.us_thread.start()

    def merge_sensor_data(self):
        sensor_data = np.full(360, self.config['max_distance'])
        for angle, distance in self.lidar_data.items():
            sensor_data[angle % 360] = distance

        for angle, distance in self.us_data.items():
            adjusted_angle = angle % 360
            sensor_data[adjusted_angle] = min(sensor_data[adjusted_angle], distance)

        return sensor_data

    def check_for_obstacles(self):
        self.start_ultrasonic_sweep()
        self.get_lidar_data()
        self.us_thread.join()  # Ensure ultrasonic data collection is complete
        return self.merge_sensor_data()

