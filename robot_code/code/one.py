import numpy as np
import time
import logging
from rplidar import RPLidar, RPLidarException

import os
import sys

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

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def calculate_midpoint(loc1, loc2):
    midpoint = ((loc1[0] + loc2[0]) / 2, (loc1[1] + loc2[1]) / 2)
    logging.info(f"Calculated midpoint: {midpoint}")
    return midpoint

class VectorFieldHistogram:
    def __init__(self, cell_size=10, threshold=300):
        self.cell_size = cell_size
        self.threshold = threshold
        logging.info("Initialized Vector Field Histogram")

    def compute_histogram(self, sensor_data):
        histogram = np.zeros(360 // self.cell_size)
        for angle in range(360):
            cell_index = angle // self.cell_size
            distance = sensor_data[angle]
            if distance < self.threshold:
                histogram[cell_index] += 1
        logging.debug("Computed histogram: {}".format(histogram))
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
        logging.info(f"Safe direction found: {best_direction} with obstacle count: {min_obstacle_count}")
        return best_direction

class RobotController:
    def __init__(self, config):
        self.robot = Robot(config)
        self.lidar_scanner = LidarScanner('/dev/ttyUSB0')
        self.obstacle_checker = ObstacleChecker(self.lidar_scanner, Ultrasonic(Pin('D8'), Pin('D9')), {'max_distance': 4000})
        self.vfh = VectorFieldHistogram()
        self.steering_threshold = 10  # Degrees within which the robot should move forward
        self.max_turn_angle = 50     # Max degrees the robot should turn at once

        self.current_loc = get_current_gps()
        self.goal_loc = (62.878868,27.637853)  # Update with actual target GPS
        logging.info(f"Current GPS: {self.current_loc}, Goal GPS: {self.goal_loc}")
        self.origin, self.scale, self.grid = self.initialize_grid(self.current_loc, self.goal_loc, 10, 10000)

        self.start_position = self.gps_to_grid(self.current_loc[0], self.current_loc[1])
        self.goal_position = self.gps_to_grid(self.goal_loc[0], self.goal_loc[1])
        logging.info(f"Start position on grid: {self.start_position}, Goal position on grid: {self.goal_position}")

        self.planned_path = None
        self.current_path_index = 0
        if self.start_position == self.goal_position:
            if np.linalg.norm(np.array(self.current_loc) - np.array(self.goal_loc)) * 111000 > 1:  # more than 1 meter apart
                logging.info("Close proximity path planning activated.")
                self.planned_path = [self.start_position, self.goal_position]
            else:
                logging.info("Start and Goal positions are identical; no path needed.")
        else:
            self.planned_path = a_star(self.start_position, self.goal_position, self.grid)
            if self.planned_path:
                logging.info("Path planned using A*")
            else:
                logging.error("A* pathfinding failed.")

    # Dynamic grid scaling based on area size for improved path precision
    def initialize_grid(self, current_loc, goal_loc, expected_range_m, base_resolution):
        actual_distance_m = np.linalg.norm(np.array(current_loc) - np.array(goal_loc)) * 111000
        actual_distance_m = max(actual_distance_m, 0.1)
        scale = base_resolution / actual_distance_m
        origin = calculate_midpoint(current_loc, goal_loc)
        grid_resolution = int(base_resolution * (expected_range_m / max(actual_distance_m, 10)))
        grid_shape = (grid_resolution, grid_resolution)
        grid = np.zeros(grid_shape)
        logging.info(f"Grid initialized with origin {origin}, scale {scale}, resolution {grid_resolution}")
        return origin, scale, grid


    def gps_to_grid(self, latitude, longitude):
        x = (longitude - self.origin[1]) * self.scale
        y = (latitude - self.origin[0]) * self.scale
        grid_x = int(round(x))
        grid_y = int(round(y))
        if 0 <= grid_x < self.grid.shape[1] and 0 <= grid_y < self.grid.shape[0]:
            logging.info(f"GPS ({latitude}, {longitude}) converted to grid position: ({grid_y}, {grid_x})")
            return (grid_y, grid_x)
        else:
            grid_x = max(0, min(self.grid.shape[1] - 1, grid_x))
            grid_y = max(0, min(self.grid.shape[0] - 1, grid_y))
            logging.warning(f"Adjusted grid position: ({grid_y}, {grid_x})")
            return (grid_y, grid_x)

    def calculate_path_direction(self):
        if self.planned_path and self.current_path_index < len(self.planned_path):
            current_position = self.gps_to_grid(*get_current_gps())
            next_position = self.planned_path[self.current_path_index]
            if next_position == current_position:
                self.current_path_index += 1
                if self.current_path_index < len(self.planned_path):
                    next_position = self.planned_path[self.current_path_index]
                else:
                    return None  # Path complete

            direction_angle = np.degrees(np.arctan2(next_position[1] - current_position[1], next_position[0] - current_position[0]))
            return (direction_angle + 360) % 360
        return None

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
                time.sleep(1)  # Control loop pause
                self.update_path_if_needed()
        except (KeyboardInterrupt, Exception) as e:
            logging.error("An error occurred: %s", e)
        finally:
            self.robot.stop()
            self.lidar_scanner.close()
            logging.info("Emergency stop! The robot and lidar scanner have been turned off.")

    # Real-time path recalculation if path deviation or new obstacles are detected
    def update_path_if_needed(self):
        current_position = self.gps_to_grid(*get_current_gps())
        if self.planned_path and current_position != self.planned_path[self.current_path_index]:
            logging.info("Recalculating path due to deviation or obstacle.")
            self.planned_path = a_star(current_position, self.goal_position, self.grid)


    def reached_goal(self):
        current_position = get_current_gps()
        goal_reached = np.linalg.norm(np.array(current_position) - np.array(self.goal_position)) < 0.0001
        logging.info(f"Current GPS position: {current_position}, Goal position: {self.goal_position}, Goal reached: {goal_reached}")
        return goal_reached

    # Adaptive robot movement strategy with smooth steering and dynamic speed adjustments
    def move_robot(self, steering_direction):
        if steering_direction is None:
            logging.info("No valid steering direction provided. Robot will not move.")
            return

        current_heading = get_current_heading()
        error = (steering_direction - current_heading + 180) % 360 - 180

        logging.info(f"Steering direction: {steering_direction}, Current heading: {current_heading}, Heading error: {error}")

        if abs(error) < self.steering_threshold:
            self.robot.forward(100)
        elif error > 0:
            turn_angle = min(self.max_turn_angle, error)
            self.robot.turn_right(70, max(0, 1 - turn_angle / 100))
        else:
            turn_angle = min(self.max_turn_angle, -error)
            self.robot.turn_left(70, max(0, 1 - turn_angle / 100))


if __name__ == "__main__":
    robot_controller = RobotController(config)
    robot_controller.main_loop()
