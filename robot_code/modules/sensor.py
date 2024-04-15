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

class KalmanFilter:
    def __init__(self):
        self.estimated_distance = 0
        self.error_estimate = 1
        self.error_measure = 1

    def update(self, measurement):
        kalman_gain = self.error_estimate / (self.error_estimate + self.error_measure)
        self.estimated_distance = self.estimated_distance + kalman_gain * (measurement - self.estimated_distance)
        self.error_estimate = (1 - kalman_gain) * self.error_estimate
        return self.estimated_distance

class SensorFusion:
    def __init__(self):
        self.kalman_filters = [KalmanFilter() for _ in range(360)]

    def fuse_data(self, lidar_data, ultrasonic_data):
        fused_data = np.full(360, float('inf'))
        for angle in range(360):
            measurement = min(lidar_data.get(angle, float('inf')), ultrasonic_data.get(angle, float('inf')))
            fused_data[int(angle)] = self.kalman_filters[int(angle)].update(measurement)
        return fused_data


class LidarScanner:
    def __init__(self, port):
        self.lidar = RPLidar(port)
        self.connected = False
        self.try_to_connect()

    def try_to_connect(self):
        try:
            self.lidar.connect()
            self.connected = True
            logging.info("LIDAR connected.")
        except RPLidarException as e:
            logging.error(f"Failed to connect to LIDAR: {e}")
            self.connected = False
            raise SystemExit(e)

    def iter_scans(self):
        if not self.connected:
            logging.error("LIDAR not connected for scanning.")
            return
        try:
            for scan in self.lidar.iter_scans():
                yield {round(measurement[1]): measurement[2] for measurement in scan if measurement[0] > 0}
        except RPLidarException as e:
            logging.error(f"Lidar scanning error: {e}")
            self.handle_lidar_error()

    def handle_lidar_error(self):
        self.close()
        self.try_to_connect()

    def close(self):
        if self.connected:
            try:
                self.lidar.stop()
                self.lidar.disconnect()
                self.connected = False
                logging.info("LIDAR disconnected safely.")
            except RPLidarException as e:
                logging.error(f"Failed to properly shutdown LIDAR: {e}")


class ObstacleChecker:
    def __init__(self, lidar, us, config):
        self.lidar = lidar
        self.us = us
        self.config = config
        self.us_thread = None
        self.lidar_data = {}
        self.us_data = {}
        self.sensor_fusion = SensorFusion()

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
        if self.us_thread.is_alive():
            self.us_thread.join()
        return self.sensor_fusion.fuse_data(self.lidar_data, self.us_data)

    def check_for_obstacles(self):
        self.start_ultrasonic_sweep()
        self.get_lidar_data()
        fused_sensor_data = self.merge_sensor_data()
        logging.debug(f"Fused Sensor Data: {fused_sensor_data}")
        return fused_sensor_data
