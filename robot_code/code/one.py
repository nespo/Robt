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
from robot_code.modules.sensor import LidarScanner
from robot_code.modules.obstacle import ObstacleChecker
from robot_code.code.config import config
from robot_code.modules.nav import get_current_gps, get_current_heading
from robot_code.modules.a_star import a_star

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def calculate_midpoint(loc1, loc2):
    midpoint = ((loc1[0] + loc2[0]) / 2, (loc1[1] + loc2[1]) / 2)
    logging.info(f"Calculated midpoint: {midpoint}")
    return midpoint

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
            if angle >= len(sensor_data):
                continue
            distance = sensor_data[angle]
            sector_index = int(angle // sector_angle)  # Explicit integer conversion
            if 0 <= sector_index < self.sectors:
                histogram[sector_index] += 1
        return histogram



    def find_safe_trajectory(self, histogram, current_heading, velocities, goal_direction):
        safe_trajectories = []
        for speed, angle in velocities:
            sector_index = (angle // (360 // self.sectors)) % self.sectors
            if histogram[sector_index] == 0:  # No obstacles in the sector
                cost = self.calculate_cost(angle, speed, goal_direction)
                safe_trajectories.append((cost, speed, angle))
        return min(safe_trajectories, default=(None, None))[1:]  # Safely return the trajectory with the least cost

    def calculate_cost(self, angle, speed, goal_direction):
        angle_cost = min((angle - goal_direction) % 360, (goal_direction - angle) % 360)
        speed_cost = max(0, 1 - speed / 100)  # Assuming max speed is 100
        return angle_cost + speed_cost


class DynamicWindowApproach:
    def __init__(self, max_speed, max_turn_rate):
        self.max_speed = max_speed
        self.max_turn_rate = max_turn_rate

    def generate_velocities(self, current_speed, current_turn_rate):
        # Generate possible velocities (speed, turn rate) within dynamic constraints
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
        self.current_speed = 0
        self.current_turn_rate = 0
        self.lidar_scanner = LidarScanner('/dev/ttyUSB0')
        self.steering_pid = PIDController(kp=0.1, ki=0.01, kd=0.05, setpoint=0)
        self.speed_pid = PIDController(kp=0.2, ki=0.02, kd=0.1, setpoint=100)
        self.obstacle_checker = ObstacleChecker(self.lidar_scanner, Ultrasonic(Pin('D8'), Pin('D9')), {'max_distance': 4000})
        self.vfh = VFHPlus(cell_size=5, threshold=250, sectors=180)
        self.dwa = DynamicWindowApproach(max_speed=100, max_turn_rate=45)
        self.steering_threshold = 10
        self.max_turn_angle = 50
        self.current_loc = get_current_gps()
        self.goal_loc = (62.878868, 27.637853)
        logging.info(f"Current GPS: {self.current_loc}, Goal GPS: {self.goal_loc}")
        self.origin, self.scale, self.grid = self.initialize_grid(self.current_loc, self.goal_loc, 10, 10000)
        self.start_position = self.gps_to_grid(self.current_loc[0], self.current_loc[1])
        self.goal_position = self.gps_to_grid(self.goal_loc[0], self.goal_loc[1])
        self.planned_path = a_star(self.start_position, self.goal_position, self.grid) if self.start_position != self.goal_position else []
        self.current_path_index = 0
        logging.info(f"Start position on grid: {self.start_position}, Goal position on grid: {self.goal_position}")

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
        # Handle out-of-bound grid positions
        grid_x = max(0, min(self.grid.shape[1] - 1, grid_x))
        grid_y = max(0, min(self.grid.shape[0] - 1, grid_y))
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
            return int((direction_angle + 360) % 360)  # Ensure the result is an integer
        return None


    def main_loop(self):
        try:
            while not self.reached_goal():
                current_heading = get_current_heading()
                sensor_data = self.obstacle_checker.check_for_obstacles()
                histogram = self.vfh.compute_histogram(sensor_data)
                if np.any(histogram > 0):
                    steering_direction, steering_speed = self.vfh.find_safe_trajectory(histogram, current_heading, self.dwa.generate_velocities(self.current_speed, self.current_turn_rate), self.calculate_path_direction())
                    if np.min(sensor_data) < 50:
                        self.reverse_and_reroute()
                    elif steering_direction is None:
                        self.halt_and_reassess()
                    else:
                        self.move_robot(steering_direction, steering_speed)
                else:
                    path_direction = self.calculate_path_direction()
                    if path_direction is not None:
                        self.move_robot(path_direction, self.speed_pid.setpoint)
                time.sleep(1)
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

    def move_robot(self, direction, speed):
        current_heading = get_current_heading()
        error = (direction - current_heading + 180) % 360 - 180
        turn_rate = self.steering_pid.compute(error)
        adjusted_speed = self.speed_pid.compute(abs(error))
        if abs(error) < self.steering_threshold:
            self.robot.forward(adjusted_speed)
        elif error > 0:
            self.robot.turn_right(turn_rate, adjusted_speed)
        else:
            self.robot.turn_left(turn_rate, adjusted_speed)

    def reverse_and_reroute(self):
        logging.info("Obstacle encountered, reversing and rerouting.")
        self.robot.reverse(50)
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
