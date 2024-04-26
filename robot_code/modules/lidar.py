import numpy as np
import logging
from rplidar import RPLidar, RPLidarException

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')


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
                logging.debug(f"Raw Scan Data: {scan}")
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

    def get_lidar_data(self):
        if not self.lidar.connected:
            logging.error("LIDAR not connected for data retrieval.")
            self.lidar_data = {}
            return
        try:
            self.lidar_data = next(self.lidar.iter_scans(), {})
            logging.debug(f"Lidar Data (after processing): {self.lidar_data}")
        except StopIteration:
            logging.error("No more LIDAR scans available.")
            self.lidar_data = {}
        except RPLidarException as e:
            logging.error(f"Error obtaining LIDAR data: {e}")
            self.lidar_data = {}


    def check_for_obstacles(self):
        self.get_lidar_data()  # Refresh lidar data
        sensor_data = {angle: distance for angle, distance in enumerate(self.lidar_data)}
        #print("sensor DATA: ", sensor_data)
        return sensor_data



lidar_scanner = LidarScanner('/dev/ttyUSB0')
obstacle_checker = ObstacleChecker(lidar_scanner)

sensor_data = obstacle_checker.check_for_obstacles()

    # Ensure all sensor data are finite and replace 'inf' and NaN with a high but finite value
    #valid_sensor_data = np.where(np.isfinite(sensor_data), sensor_data, 1000)
    #print("Valid Sensor Data:", valid_sensor_data)  # Debug print

print(sensor_data)