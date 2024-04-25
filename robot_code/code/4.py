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

class VFHPlus:
    def __init__(self, cell_size=10, threshold=300, sectors=180, safety_distance=100):
        self.cell_size = cell_size
        self.threshold = threshold
        self.sectors = sectors
        self.safety_distance = safety_distance
        logging.info("Initialized VFH+ with configured parameters.")

    def compute_histogram(self, sensor_data):
        histogram = np.zeros(self.sectors)
        sector_angle = 360 // self.sectors
        for angle in range(360):
            if angle >= len(sensor_data) or sensor_data[angle] >= self.safety_distance:
                continue
            sector_index = angle // sector_angle
            histogram[sector_index] += 1
        return histogram

    def find_safe_direction(self, histogram, current_heading):
        best_direction, min_obstacle_count = None, float('inf')
        for i, count in enumerate(histogram):
            if count < min_obstacle_count:
                min_obstacle_count = count
                best_direction = i * self.cell_size + (self.cell_size // 2)

        if best_direction is not None:
            best_direction = (best_direction - current_heading + 360) % 360
            if best_direction > 180:
                best_direction -= 360
        return best_direction

class DynamicWindowApproach:
    def __init__(self, max_speed, max_turn_rate):
        self.max_speed = max_speed
        self.max_turn_rate = max_turn_rate

    def generate_velocities(self, current_speed, current_turn_rate):
        speeds = np.linspace(max(0, current_speed - 10), min(self.max_speed, current_speed + 10), 5)
        turn_rates = np.linspace(max(-self.max_turn_rate, current_turn_rate - 10), min(self.max_turn_rate, current_turn_rate + 10), 5)
        return [(s, tr) for s in speeds for tr in turn_rates]

class PIDController:
    def __init__(self, kp, ki, kd, setpoint):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.previous_error = 0
        self.integral = 0
        self.dt = 1  # time interval in seconds

    def compute(self, measurement):
        error = self.setpoint - measurement
        self.integral += error * self.dt
        derivative = (error - self.previous_error) / self.dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output

class RobotController:
    def __init__(self, config):
        self.robot = Robot(config)
        self.lidar_scanner = LidarScanner('/dev/ttyUSB0')
        self.obstacle_checker = ObstacleChecker(self.lidar_scanner, Ultrasonic(Pin('D8'), Pin('D9')), {'max_distance': 4000})
        self.vfh = VFHPlus(cell_size=5, threshold=250, sectors=180, safety_distance=50)
        self.steering_pid = PIDController(kp=0.1, ki=0.01, kd=0.05, setpoint=0)
        self.speed_pid = PIDController(kp=0.2, ki=0.02, kd=0.1, setpoint=100)
        self.current_loc = get_current_gps()
        self.goal_loc = (62.878815, 27.637536)
        self.origin, self.scale, self.grid = self.initialize_grid(self.current_loc, self.goal_loc, 10, 10000)
        self.start_position = self.gps_to_grid(self.current_loc[0], self.current_loc[1])
        self.goal_position = self.gps_to_grid(self.goal_loc[0], self.goal_loc[1])
        self.planned_path = a_star(self.start_position, self.goal_position, self.grid) if self.start_position != self.goal_position else []
        self.current_path_index = 0
        logging.info(f"Start position on grid: {self.start_position}, Goal position on grid: {self.goal_position}")

    def initialize_grid(self, current_loc, goal_loc, expected_range_m, base_resolution):
        actual_distance_m = np.linalg.norm(np.array(current_loc) - np.array(goal_loc)) * 111000
        scale = base_resolution / actual_distance_m
        origin = calculate_midpoint(current_loc, goal_loc)
        grid_resolution = int(base_resolution * (expected_range_m / actual_distance_m))
        grid_shape = (grid_resolution, grid_resolution)
        grid = np.zeros(grid_shape)
        return origin, scale, grid

    def gps_to_grid(self, latitude, longitude):
        x = (longitude - self.origin[1]) * self.scale
        y = (latitude - self.origin[0]) * self.scale
        grid_x = int(round(x))
        grid_y = int(round(y))
        grid_x = max(0, min(self.grid.shape[1] - 1, grid_x))
        grid_y = max(0, min(self.grid.shape[0] - 1, grid_y))
        return (grid_y, grid_x)

    def main_loop(self):
        try:
            while not self.reached_goal():
                current_heading = get_current_heading()
                sensor_data = self.obstacle_checker.check_for_obstacles()
                valid_sensor_data = np.where(np.isfinite(sensor_data), sensor_data, 1000)
                histogram = self.vfh.compute_histogram(valid_sensor_data)
                if np.any(histogram > 0):
                    steering_direction = self.vfh.find_safe_direction(histogram, current_heading)
                else:
                    steering_direction = self.calculate_path_direction()
                self.move_robot(steering_direction)
                time.sleep(1)  # Control loop pause for stability
                self.update_path_if_needed()
        except (KeyboardInterrupt, Exception) as e:
            logging.error(f"An error occurred: {e}")
            self.robot.stop()
            self.lidar_scanner.close()
            logging.info("Emergency stop! The robot and lidar scanner have been turned off.")

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

    def move_robot(self, steering_direction):
        if steering_direction is None:
            logging.info("No valid steering direction provided. Robot will not move.")
            return  # No valid direction to move

        current_heading = get_current_heading()
        error = steering_direction - current_heading

        # Normalize the error to be within the range [-180, 180]
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        logging.info(f"Steering direction: {steering_direction}, Current heading: {current_heading}, Heading error: {error}")

        steering_threshold = 10  # degrees within which the robot should move forward
        max_turn_angle = 50  # max degrees the robot should turn at once

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

    def reverse_and_reroute(self):
        logging.info("Obstacle encountered, reversing and rerouting.")
        self.robot.backward(50)
        time.sleep(2)  # Reverse for 2 seconds
        self.update_path_if_needed()

    def halt_and_reassess(self):
        logging.info("Halt and reassessing situation.")
        self.robot.stop()
        time.sleep(5)  # Wait and reassess
        self.update_path_if_needed()

    def emergency_stop(self):
        logging.info("Emergency stop triggered.")
        self.robot.stop()

if __name__ == "__main__":
    robot_controller = RobotController(config)
    robot_controller.main_loop()
