import numpy as np
import time
import logging
from threading import Thread
from rplidar import RPLidar, RPLidarException

import os
import sys

# Ensure correct library import paths
script_dir = os.path.dirname(os.path.realpath(__file__))
parent_dir = os.path.join(script_dir, '..', '..')
sys.path.append(os.path.abspath(parent_dir))

# Import robot specific modules
from robot_code.code.motor_control import Robot
from robot_code.modules.ultrasonic import Ultrasonic
from robot_code.modules.pin import Pin
from robot_code.modules.obstacle import LidarScanner, ObstacleChecker
from robot_code.code.config import config
from robot_code.modules.navigation import get_current_gps, get_current_heading
from robot_code.modules.a_star import a_star

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def calculate_midpoint(loc1, loc2):
    return ((loc1[0] + loc2[0]) / 2, (loc1[1] + loc2[1]) / 2)

class VectorFieldHistogram:
    def __init__(self, cell_size=10, threshold=300):
        self.cell_size = cell_size
        self.threshold = threshold

    def compute_histogram(self, sensor_data):
        histogram = np.zeros(360 // self.cell_size)
        for angle in range(360):
            cell_index = angle // self.cell_size
            distance = sensor_data[angle]
            if distance < self.threshold:
                histogram[cell_index] += 1
        return histogram

    def find_safe_direction(self, histogram, current_heading):
        best_direction, min_obstacle_count = None, float('inf')
        for i, count in enumerate(histogram):
            if count < min_obstacle_count:
                min_obstacle_count = count
                best_direction = i * self.cell_size + (self.cell_size // 2)

        if best_direction is not None:
            best_direction = (best_direction - current_heading) % 360
            if best_direction > 180:
                best_direction -= 360

        return best_direction
    
class RobotController:
    def __init__(self, config):
        self.robot = Robot(config)
        self.lidar_scanner = LidarScanner('/dev/ttyUSB0')
        self.obstacle_checker = ObstacleChecker(self.lidar_scanner, Ultrasonic(Pin('D8'), Pin('D9')), {'max_distance': 4000})
        self.vfh = VectorFieldHistogram()
        
        self.current_loc = get_current_gps()
        self.goal_loc = (62.878866, 27.637739)
        self.origin, self.scale, self.grid = self.initialize_grid(self.current_loc, self.goal_loc, 1000, 2000)
        
        self.start_position = self.gps_to_grid(self.current_loc[0], self.current_loc[1])
        self.goal_position = self.gps_to_grid(self.goal_loc[0], self.goal_loc[1])
        
        if self.start_position and self.goal_position:
            self.planned_path = a_star(self.start_position, self.goal_position, self.grid)
        else:
            self.planned_path = None
            logging.error("Invalid start or goal position for A* algorithm.")

    def initialize_grid(self, current_loc, goal_loc, expected_range_m, grid_resolution):
        origin = calculate_midpoint(current_loc, goal_loc)
        scale = grid_resolution / expected_range_m
        grid_shape = (grid_resolution, grid_resolution)
        grid = np.zeros(grid_shape)
        return origin, scale, grid

    def gps_to_grid(self, latitude, longitude):
        x = int((longitude - self.origin[1]) * self.scale)
        y = int((latitude - self.origin[0]) * self.scale)
        if 0 <= x < self.grid.shape[1] and 0 <= y < self.grid.shape[0]:
            return (y, x)
        else:
            logging.error("Adjusted GPS coordinates out of grid bounds: %f, %f", latitude, longitude)
            x = max(0, min(self.grid.shape[1] - 1, x))
            y = max(0, min(self.grid.shape[0] - 1, y))
            return (y, x)

    def calculate_path_direction(self):
        if self.current_path_index < len(self.planned_path):
            next_position = self.planned_path[self.current_path_index]
            current_position = self.gps_to_grid(*get_current_gps())
            if next_position == current_position:
                self.current_path_index += 1
                if self.current_path_index < len(self.planned_path):
                    next_position = self.planned_path[self.current_path_index]

            direction_angle = np.degrees(np.arctan2(next_position[0] - current_position[0], next_position[1] - current_position[1]))
            return (direction_angle + 360) % 360
        return None  # Path completed or error

    def main_loop(self):
        try:
            while not self.reached_goal() and self.planned_path:
                current_heading = get_current_heading()
                sensor_data = self.obstacle_checker.check_for_obstacles()
                histogram = self.vfh.compute_histogram(sensor_data)
                
                if np.any(histogram > 0): 
                    steering_direction = self.vfh.find_safe_direction(histogram, current_heading)
                else:
                    steering_direction = self.calculate_path_direction()

                self.move_robot(steering_direction)
                time.sleep(1)  # Control loop pause for stability
        except (KeyboardInterrupt, Exception) as e:
            logging.error("An error occurred: %s", e)
            self.robot.stop()
            self.lidar_scanner.close()


    def reached_goal(self):
        if not self.planned_path:
            return True
        current_position = get_current_gps()
        return np.linalg.norm(np.array(current_position) - np.array(self.goal_position)) < 0.0001
    
    def move_robot(self, steering_direction):
        if steering_direction is None:
            return  # No valid direction to move

        error = steering_direction - get_current_heading()
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        if error > 10:
            self.robot.turn_right(min(50, error))
        elif error < -10:
            self.robot.turn_left(min(50, -error))
        else:
            self.robot.forward(50)

if __name__ == "__main__":
    robot_controller = RobotController(config)
    robot_controller.main_loop()
