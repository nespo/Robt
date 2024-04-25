import numpy as np
import time
import logging
from rplidar import RPLidar
import os
import sys

# Correct library import paths
script_dir = os.path.dirname(os.path.realpath(__file__))
parent_dir = os.path.join(script_dir, '..', '..')
sys.path.append(os.path.abspath(parent_dir))

from robot_code.code.motor_control import Robot
from robot_code.modules.ultrasonic import Ultrasonic
from robot_code.modules.pin import Pin
from robot_code.modules.sensor import LidarScanner, ObstacleChecker
from config import config
from robot_code.modules.nav import get_current_gps, get_current_heading
from robot_code.modules.a_star import a_star

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def calculate_midpoint(loc1, loc2):
    return ((loc1[0] + loc2[0]) / 2, (loc1[1] + loc2[1]) / 2)

class VFHPlus:
    def __init__(self, cell_size=10, threshold=300, sectors=180):
        self.cell_size = cell_size
        self.threshold = threshold
        self.sectors = sectors
        logging.info("Initialized VFH+ with finer resolution.")

    def compute_histogram(self, sensor_data):
        histogram = np.zeros(self.sectors)
        sector_angle = 360 // self.sectors
        for angle in range(360):
            distance = sensor_data[angle % len(sensor_data)]
            if distance > 0:  # Check for valid distances
                sector_index = int(angle // sector_angle)
                histogram[sector_index] += 1
        # Smooth the histogram to reduce noise sensitivity
        histogram = np.convolve(histogram, np.ones(5)/5, mode='same')
        return histogram

    def find_safe_direction(self, histogram, current_heading):
        best_direction, min_obstacle_count = None, float('inf')
        for i, count in enumerate(histogram):
            if count < min_obstacle_count:
                min_obstacle_count = count
                best_direction = i * (360 / self.sectors)
        return (best_direction - current_heading) % 360

class RobotController:
    def __init__(self, config):
        self.robot = Robot(config)
        self.lidar = LidarScanner('/dev/ttyUSB0')
        self.ultrasonic = Ultrasonic(Pin('D8'), Pin('D9'))
        self.obstacle_checker = ObstacleChecker(self.lidar, self.ultrasonic, config)
        self.vfh = VFHPlus()
        self.last_path_update_time = 0
        self.path_update_interval = 10  # seconds
        self.current_path = []
        self.goal_loc = (62.878815, 27.637536)  # Example goal

    def main_loop(self):
        try:
            while not self.reached_goal():
                current_position = get_current_gps()
                current_heading = get_current_heading()
                lidar_data = self.lidar.get_scan()
                histogram = self.vfh.compute_histogram(lidar_data)
                safe_direction = self.vfh.find_safe_direction(histogram, current_heading)

                # Path update logic
                if time.time() - self.last_path_update_time > self.path_update_interval or self.need_path_update(current_position):
                    self.current_path = a_star(current_position, self.goal_loc, self.obstacle_checker.get_grid())
                    self.last_path_update_time = time.time()

                self.move_robot(safe_direction)
                time.sleep(1)
        finally:
            self.robot.stop()

    def move_robot(self, direction):
        if direction is None:
            self.robot.stop()
        elif direction < -10:
            self.robot.turn_left(abs(direction))
        elif direction > 10:
            self.robot.turn_right(direction)
        else:
            self.robot.forward(100)  # Move forward at full speed

    def reached_goal(self):
        current_position = get_current_gps()
        return np.linalg.norm(np.array(current_position) - np.array(self.goal_loc)) < 0.05  # 5 cm threshold

    def need_path_update(self, current_position):
        # Check if the robot is off the current path
        return np.any([np.linalg.norm(np.array(current_position) - np.array(point)) > 1 for point in self.current_path])  # 1m threshold

if __name__ == "__main__":
    #config = {'max_distance': 4000}  # Example configuration
    print(config)
    robot_controller = RobotController(config)
    robot_controller.main_loop()
