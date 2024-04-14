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
from robot_code.modules.nav import get_current_gps, get_current_heading
from robot_code.modules.a_star import a_star

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def calculate_midpoint(loc1, loc2):
    midpoint = ((loc1[0] + loc2[0]) / 2, (loc1[1] + loc2[1]) / 2)
    print(f"Calculated midpoint: {midpoint}")
    return midpoint

class VectorFieldHistogram:
    def __init__(self, cell_size=10, threshold=300):
        self.cell_size = cell_size
        self.threshold = threshold
        print("Initialized Vector Field Histogram")

    def compute_histogram(self, sensor_data):
        histogram = np.zeros(360 // self.cell_size)
        for angle in range(360):
            cell_index = angle // self.cell_size
            distance = sensor_data[angle]
            if distance < self.threshold:
                histogram[cell_index] += 1
        print("Computed histogram:", histogram)
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
        print(f"Safe direction found: {best_direction} with obstacle count: {min_obstacle_count}")
        return best_direction
    
class RobotController:
    def __init__(self, config):
        self.robot = Robot(config)
        self.lidar_scanner = LidarScanner('/dev/ttyUSB0')
        self.obstacle_checker = ObstacleChecker(self.lidar_scanner, Ultrasonic(Pin('D8'), Pin('D9')), {'max_distance': 4000})
        self.vfh = VectorFieldHistogram()
        
        self.current_loc = get_current_gps()
        self.goal_loc = (62.8794233,27.6316472)
        print(f"Current GPS: {self.current_loc}, Goal GPS: {self.goal_loc}")
        self.origin, self.scale, self.grid = self.initialize_grid(self.current_loc, self.goal_loc, 10, 10000)
        
        self.start_position = self.gps_to_grid(self.current_loc[0], self.current_loc[1])
        self.goal_position = self.gps_to_grid(self.goal_loc[0], self.goal_loc[1])
        print(f"Start position on grid: {self.start_position}, Goal position on grid: {self.goal_position}")
        
        if self.start_position and self.goal_position and self.start_position != self.goal_position:
            self.planned_path = a_star(self.start_position, self.goal_position, self.grid)
            print("Path planned using A*")
        else:
            self.planned_path = None
            if self.start_position == self.goal_position:
                print("Start and Goal positions are identical; no path needed.")
            else:
                logging.error("Invalid start or goal position for A* algorithm.")


    def initialize_grid(self, current_loc, goal_loc, expected_range_m, grid_resolution):
    # Dynamically adjust the grid resolution based on the actual distance
        actual_distance_m = np.linalg.norm(np.array(current_loc) - np.array(goal_loc)) * 111000  # Convert degrees to meters roughly
        if actual_distance_m < expected_range_m:
            scale = grid_resolution / actual_distance_m  # Increase scale for finer resolution
        else:
            scale = grid_resolution / expected_range_m

        origin = calculate_midpoint(current_loc, goal_loc)
        grid_shape = (grid_resolution, grid_resolution)
        grid = np.zeros(grid_shape)
        print(f"Grid initialized with origin {origin}, scale {scale}")
        return origin, scale, grid


    def gps_to_grid(self, latitude, longitude):
        x = (longitude - self.origin[1]) * self.scale
        y = (latitude - self.origin[0]) * self.scale
        # Convert to meters or another unit consistent with your grid resolution
        grid_x = int(round(x))  # Round to the nearest grid cell after scaling
        grid_y = int(round(y))
        if 0 <= grid_x < self.grid.shape[1] and 0 <= grid_y < self.grid.shape[0]:
            print(f"GPS ({latitude}, {longitude}) converted to precise grid position: ({grid_y}, {grid_x})")
            return (grid_y, grid_x)
        else:
            # Handle out of bounds goal position
            grid_x = max(0, min(self.grid.shape[1] - 1, grid_x))
            grid_y = max(0, min(self.grid.shape[0] - 1, grid_y))
            print(f"Adjusted grid position: ({grid_y}, {grid_x})")
            return (grid_y, grid_x)



    def calculate_path_direction(self):
        if self.current_path_index < len(self.planned_path):
            next_position = self.planned_path[self.current_path_index]
            current_position = self.gps_to_grid(*get_current_gps())
            if next_position == current_position:
                self.current_path_index += 1
                if self.current_path_index < len(self.planned_path):
                    next_position = self.planned_path[self.current_path_index]

            direction_angle = np.degrees(np.arctan2(next_position[0] - current_position[0], next_position[1] - current_position[1]))
            print("Direction_angle: ", direction_angle)
            return (direction_angle + 360) % 360
        return None  # Path completed or error

    def main_loop(self):
        try:
            while not self.reached_goal() and self.planned_path:
                current_heading = get_current_heading()
                print(f"Current heading: {current_heading}")
                sensor_data = self.obstacle_checker.check_for_obstacles()
                histogram = self.vfh.compute_histogram(sensor_data)
                print(f"Obstacle histogram: {histogram}")

                if np.any(histogram > 0):
                    steering_direction = self.vfh.find_safe_direction(histogram, current_heading)
                    print(f"Obstacle detected, steering direction: {steering_direction}")
                else:
                    steering_direction = self.calculate_path_direction()
                    print(f"No obstacles detected, following path, steering direction: {steering_direction}")

                self.move_robot(steering_direction)
                print(f"Moving robot towards: {steering_direction}")
                time.sleep(1)  # Control loop pause for stability
        except (KeyboardInterrupt, Exception) as e:
            logging.error("An error occurred: %s", e)
        finally:
            self.robot.stop()
            self.lidar_scanner.close()
            print("Emergency stop! The robot and lidar scanner have been turned off.")

    def reached_goal(self):
        if not self.planned_path:
            print("No planned path. Goal is considered reached.")
            return True
        current_position = get_current_gps()
        goal_reached = np.linalg.norm(np.array(current_position) - np.array(self.goal_position)) < 0.0001
        print(f"Current GPS position: {current_position}, Goal position: {self.goal_position}, Goal reached: {goal_reached}")
        return goal_reached

    def move_robot(self, steering_direction):
        if steering_direction is None:
            logging.info("No valid steering direction provided. Robot will not move.")
            return  # No valid direction to move

        # Retrieve the current heading from the navigation module
        current_heading = get_current_heading()
        error = steering_direction - current_heading

        # Normalize the error to be within the range [-180, 180]
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        logging.info(f"Steering direction: {steering_direction}, Current heading: {current_heading}, Heading error: {error}")

        # Define threshold for action
        steering_threshold = 10  # degrees within which the robot should move forward
        max_turn_angle = 50  # max degrees the robot should turn at once

        # Determine movement based on error magnitude and direction
        if abs(error) < steering_threshold:
            logging.info("Moving forward")
            self.robot.forward(100)  # Move forward with power level 50
        elif error > 0:
            turn_angle = min(max_turn_angle, error)  # Limit the turn to max_turn_angle
            logging.info(f"Turning right: {turn_angle} degrees")
            self.robot.turn_right(70, 1 - turn_angle / 100)  # Reduce inside wheel speed based on turn angle
        else:
            turn_angle = min(max_turn_angle, -error)  # Limit the turn to max_turn_angle
            logging.info(f"Turning left: {turn_angle} degrees")
            self.robot.turn_left(70, 1 - turn_angle / 100)  # Reduce inside wheel speed based on turn angle


if __name__ == "__main__":
    robot_controller = RobotController(config)
    robot_controller.main_loop()
