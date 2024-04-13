import numpy as np
import time
import logging
from threading import Thread
from rplidar import RPLidar, RPLidarException

import os, sys
# Ensure correct library import paths
script_dir = os.path.dirname(__file__)
parent_dir = os.path.join(script_dir, '..', '..')
sys.path.append(os.path.abspath(parent_dir))

# Importing robot specific modules
from robot_code.code.motor_control import Robot
from robot_code.modules.ultrasonic import Ultrasonic
from robot_code.modules.pin import Pin
from robot_code.modules.obstacle import LidarScanner, ObstacleChecker
from robot_code.code.config import config
from robot_code.modules.a_star import a_star

# Importing navigation utilities
from robot_code.modules.navigation import get_current_gps, get_current_heading  

# Setup enhanced logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')


class VectorFieldHistogram:
    def __init__(self):
        self.cell_size = 10
        self.threshold = 300

    def compute_histogram(self, sensor_data):
        histogram = np.zeros(360 // self.cell_size)
        for angle in range(360):
            cell_index = angle // self.cell_size
            if sensor_data[angle] < self.threshold:
                histogram[cell_index] += 1
        return histogram

    def find_safe_direction(self, histogram, current_heading):
        best_direction = None
        min_obstacle_count = float('inf')
        for i, count in enumerate(histogram):
            if count < min_obstacle_count:
                min_obstacle_count = count
                best_direction = i * self.cell_size + (self.cell_size // 2)
        # Align the best direction with the robot's current heading
        if best_direction is not None:
            if abs(best_direction - current_heading) > 180:
                best_direction -= 360
        return best_direction
    
class RobotController:
    def __init__(self, config):
        self.robot = Robot(config)
        self.lidar_scanner = LidarScanner('/dev/ttyUSB0')
        self.obstacle_checker = ObstacleChecker(self.lidar_scanner, Ultrasonic(Pin('trig_pin'), Pin('echo_pin')), {'max_distance': 4000})
        self.vfh = VectorFieldHistogram()
        self.scale = 221744
        self.origin = (62.89238, 27.67703)
        self.start = get_current_gps()
        self.start_position = self.gps_to_grid(self.start['latitude'], self.start['longitude'])
        self.goal = (34.052235, -118.243683) 
        self.goal_position = self.gps_to_grid(self.goal[0], self.goal[1])
        self.grid = self.initialize_grid()
        self.planned_path = a_star(self.start_position, self.goal_position, self.grid)

    def initialize_grid(self):
        # Placeholder for grid initialization
        return np.zeros((2000, 2000))  # Dummy grid for simplification

    def gps_to_grid(self, latitude, longitude):
        x = int((longitude - self.origin[1]) * self.scale)
        y = int((latitude - self.origin[0]) * self.scale)
        if 0 <= x < self.grid.shape[1] and 0 <= y < self.grid.shape[0]:
            return (y, x)
        else:
            logging.error("GPS coordinates out of grid bounds: (%f, %f)", latitude, longitude)
            raise ValueError("GPS coordinates out of grid bounds")

    def main_loop(self):
        try:
            while not self.reached_goal():
                current_heading = get_current_heading()
                sensor_data = self.obstacle_checker.check_for_obstacles()

                histogram = self.vfh.compute_histogram(sensor_data)
                steering_direction = self.vfh.find_safe_direction(histogram, current_heading)

                self.move_robot(steering_direction)
                time.sleep(1)  # Control loop pause for stability

        except (KeyboardInterrupt, Exception) as e:
            logging.error("An error occurred: %s", e)
            print("Shutting down robot...")
            self.robot.stop()
            self.lidar_scanner.close()

    def reached_goal(self):
        current_position = get_current_gps()
        return np.linalg.norm(np.array([current_position[0], current_position[1]]) - np.array(self.goal_position)) < 0.0001

    def move_robot(self, steering_direction):
        if steering_direction < -10:
            self.robot.turn_left(50)
        elif steering_direction > 10:
            self.robot.turn_right(50)
        else:
            self.robot.forward(50)


if __name__ == "__main__":
    robot_controller = RobotController(config)
    robot_controller.main_loop()
