import numpy as np
import time
import logging
from threading import Thread
from rplidar import RPLidar, RPLidarException

# Assuming __file__ and other relative path initializations are handled correctly above this snippet
import os, sys
script_dir = os.path.dirname(__file__)
parent_dir = os.path.join(script_dir, '..', '..')
sys.path.append(os.path.abspath(parent_dir))

from robot_code.modules.pin import Pin
from robot_code.modules.ultrasonic import Ultrasonic

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

class LidarScanner:
    def __init__(self, port):
        self.lidar = RPLidar(port)
        self.connected = False
        try:
            self.lidar.connect()
            self.connected = True
            logging.info("LIDAR connected.")
        except RPLidarException as e:
            logging.error(f"Failed to connect to LIDAR: {e}")
            raise SystemExit(e)

    def iter_scans(self):
        if not self.connected:
            logging.error("LIDAR not connected.")
            return
        try:
            for scan in self.lidar.iter_scans():
                scan_data = {round(measurement[1]): measurement[2] for measurement in scan if measurement[0] > 0}
                yield scan_data
        except RPLidarException as e:
            logging.error(f"Lidar scanning error: {e}")
            self.close()
            raise

    def close(self):
        if self.connected:
            self.lidar.stop()
            self.lidar.disconnect()
            self.connected = False
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
        if not self.lidar.connected:
            logging.error("LIDAR not connected for data retrieval.")
            self.lidar_data = {}
            return
        try:
            self.lidar_data = next(self.lidar.iter_scans(), {})
            logging.debug(f"Lidar Data: {self.lidar_data}")
        except StopIteration:
            logging.error("No more LIDAR scans available.")
            self.lidar_data = {}
        except RPLidarException as e:
            logging.error(f"Error obtaining LIDAR data: {e}")
            self.lidar_data = {}

    def full_ultrasonic_sweep(self):
        self.us_data = {}
        for angle in range(-self.us.ANGLE_RANGE // 2, self.us.ANGLE_RANGE // 2 + 1, self.us.STEP):
            distance = self.us.get_distance_at(angle)
            if distance is not None:
                self.us_data[angle] = distance
            time.sleep(self.us.SERVO_SET_DELAY)

    def start_ultrasonic_sweep(self):
        if self.us_thread and self.us_thread.is_alive():
            self.us_thread.join()
        self.us_thread = Thread(target=self.full_ultrasonic_sweep)
        self.us_thread.start()

    def merge_sensor_data(self):
        sensor_data = np.full(360, self.config['max_distance'], dtype=np.float32)
        weights = {'lidar': 0.7, 'ultrasonic': 0.3}
        
        if self.us_thread.is_alive():
            self.us_thread.join()

        for angle in range(360):
            lidar_distance = self.lidar_data.get(angle, self.config['max_distance'])
            ultrasonic_distance = self.us_data.get(angle, self.config['max_distance'])
            sensor_data[angle] = (lidar_distance * weights['lidar'] + ultrasonic_distance * weights['ultrasonic']) / (weights['lidar'] + weights['ultrasonic'])

        logging.debug(f"Merged Sensor Data: {sensor_data}")
        return sensor_data

    def check_for_obstacles(self):
        self.start_ultrasonic_sweep()
        self.get_lidar_data()
        return self.merge_sensor_data()
