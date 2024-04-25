import numpy as np
import time
import logging
from rplidar import RPLidar, RPLidarException

import os
import sys

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Correct library import paths
script_dir = os.path.dirname(os.path.realpath(__file__))
parent_dir = os.path.join(script_dir, '..', '..')
sys.path.append(os.path.abspath(parent_dir))

# Import robot-specific modules
from robot_code.code.motor_control import Robot
from robot_code.modules.ultrasonic import Ultrasonic
from robot_code.modules.pin import Pin
from robot_code.modules.sensor import LidarScanner, ObstacleChecker
from robot_code.code.config import config
from robot_code.modules.nav import get_current_gps, get_current_heading
from robot_code.modules.a_star import a_star

class VFHPlus:
    def __init__(self, cell_size=10, threshold=300, sectors=180):
        self.cell_size = cell_size
        self.threshold = threshold
        self.sectors = sectors
        logging.info("Initialized VFH+ with finer resolution.")

    def compute_histogram(self, sensor_data):
        histogram = np.zeros(self.sectors)
        sector_angle = 360 // self.sectors
        for distance in sensor_data:
            if distance == 1000:  # Ignore max range readings
                continue
            angle = sensor_data.index(distance)  # Assuming sensor data contains angles as indices
            sector_index = angle // sector_angle
            histogram[sector_index] += 1
        return histogram

    def find_safe_direction(self, histogram, current_heading):
        best_direction = None
        min_obstacle_count = min(histogram)
        candidate_directions = [i for i, count in enumerate(histogram) if count == min_obstacle_count]
        # Select direction closest to the current heading
        best_direction = min(candidate_directions, key=lambda x: abs(x * self.cell_size + (self.cell_size // 2) - current_heading))
        return best_direction * self.cell_size + (self.cell_size // 2)

class RobotController:
    def __init__(self, config):
        self.robot = Robot(config)
        self.lidar_scanner = LidarScanner('/dev/ttyUSB0')
        self.ultrasonic = Ultrasonic(Pin('D8'), Pin('D9'))
        self.obstacle_checker = ObstacleChecker(self.lidar_scanner, self.ultrasonic, {'max_distance': 4000})
        self.vfh = VFHPlus(cell_size=5, threshold=250, sectors=180)
        self.current_loc = get_current_gps()
        self.goal_loc = (62.878815, 27.637536)
        self.initialize_grid()

    def initialize_grid(self):
        midpoint = self.calculate_midpoint(self.current_loc, self.goal_loc)
        self.origin = midpoint
        distance = np.linalg.norm(np.array(self.current_loc) - np.array(self.goal_loc)) * 111000
        self.scale = 10000 / max(distance, 1)
        self.grid = np.zeros((int(self.scale * 100), int(self.scale * 100)))
        self.update_path()

    def calculate_midpoint(self, loc1, loc2):
        return ((loc1[0] + loc2[0]) / 2, (loc1[1] + loc2[1]) / 2)

    def update_path(self):
        self.start_position = self.gps_to_grid(self.current_loc)
        self.goal_position = self.gps_to_grid(self.goal_loc)
        self.planned_path = a_star(self.start_position, self.goal_position, self.grid)
        self.current_path_index = 0

    def gps_to_grid(self, gps_coords):
        x = int((gps_coords[1] - self.origin[1]) * self.scale)
        y = int((gps_coords[0] - self.origin[0]) * self.scale)
        return (y, x)

    def main_loop(self):
        try:
            while not self.reached_goal():
                self.navigate()
                time.sleep(1)  # Pause for system stability
        except KeyboardInterrupt:
            self.robot.stop()
            self.lidar_scanner.close()
            logging.info("Emergency stop triggered.")

    def navigate(self):
        current_heading = get_current_heading()
        sensor_data = self.obstacle_checker.check_for_obstacles()
        valid_data = np.where(np.isfinite(sensor_data), sensor_data, 1000)
        histogram = self.vfh.compute_histogram(valid_data)
        safe_direction = self.vfh.find_safe_direction(histogram, current_heading)
        if safe_direction:
            self.adjust_heading(safe_direction, current_heading)
        else:
            self.follow_path()

    def adjust_heading(self, desired_heading, current_heading):
        error = desired_heading - current_heading
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        if abs(error) < 10:
            self.robot.forward(50)
        elif error > 0:
            self.robot.turn_right(70, error)
        else:
            self.robot.turn_left(70, -error)

    def follow_path(self):
        if self.current_path_index < len(self.planned_path):
            next_pos = self.planned_path[self.current_path_index]
            if self.at_position(next_pos):
                self.current_path_index += 1
            if self.current_path_index < len(self.planned_path):
                next_pos = self.planned_path[self.current_path_index]
                current_heading = get_current_heading()
                desired_heading = np.degrees(np.arctan2(next_pos[1] - self.start_position[1], next_pos[0] - self.start_position[0]))
                self.adjust_heading(desired_heading, current_heading)

    def at_position(self, position):
        current_position = self.gps_to_grid(get_current_gps())
        return current_position == position

    def reached_goal(self):
        current_position = get_current_gps()
        goal_distance = np.linalg.norm(np.array(current_position) - np.array(self.goal_loc)) * 111000  # Convert to meters
        return goal_distance < 1

if __name__ == "__main__":
    robot_controller = RobotController(config)
    robot_controller.main_loop()
