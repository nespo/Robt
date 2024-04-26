import numpy as np
import logging
from rplidar import RPLidar, RPLidarException

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

    def fuse_lidar_data(self, lidar_data):
        fused_data = np.full(360, np.inf)  # Initialize with 'inf'
        for angle in range(360):
            lidar_value = lidar_data.get(angle, np.inf)
            if lidar_value < np.inf:
                fused_data[angle] = self.kalman_filters[angle].update(lidar_value)
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
            self.close()
            self.try_to_connect()

    def close(self):
        if self.connected:
            self.lidar.stop()
            self.lidar.disconnect()
            self.connected = False
            logging.info("LIDAR disconnected safely.")

class ObstacleChecker:
    def __init__(self, lidar):
        self.lidar = lidar
        self.lidar_data = {}
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

    def merge_sensor_data(self):
        fused_data = self.sensor_fusion.fuse_lidar_data(self.lidar_data)
        print(fused_data)
        return fused_data

    def check_for_obstacles(self):
        self.get_lidar_data()  # Refresh lidar data
        fused_data = self.sensor_fusion.fuse_lidar_data(self.lidar_data)
        print("FUSED DATA: ", fused_data)
        return fused_data
