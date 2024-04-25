import numpy as np
import time
import logging
import threading
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
    def __init__(self, cell_size=10, threshold=300, sectors=180):
        self.cell_size = cell_size
        self.threshold = threshold
        self.sectors = sectors
        logging.info("Initialized VFH+ with finer resolution.")

    def compute_histogram(self, sensor_data):
        print("Inside histogram")
        histogram = np.zeros(self.sectors)
        sector_angle = 360 // self.sectors
        for angle in range(360):
            if angle >= len(sensor_data):
                continue
            sector_index = int(angle // sector_angle)
            if 0 <= sector_index < self.sectors:
                histogram[sector_index] += 1
        print(histogram)
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

class DynamicWindowApproach:
    def __init__(self, max_speed, max_turn_rate):
        self.max_speed = max_speed
        self.max_turn_rate = max_turn_rate

    def generate_velocities(self, current_speed, current_turn_rate):
        speeds = np.linspace(max(0, current_speed - 10), min(self.max_speed, current_speed + 10), 5)
        turn_rates = np.linspace(max(-self.max_turn_rate, current_turn_rate - 10), min(self.max_turn_rate, current_turn_rate + 10), 5)
        return [(s, tr) for s in speeds for tr in turn_rates]


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
        self.current_loc = get_current_gps()
        self.goal_loc = (62.878815, 27.637536)
        self.origin, self.scale, self.grid = self.initialize_grid(self.current_loc, self.goal_loc, 10, 10000)
        self.start_position = self.gps_to_grid(self.current_loc[0], self.current_loc[1])
        self.goal_position = self.gps_to_grid(self.goal_loc[0], self.goal_loc[1])
        self.planned_path = a_star(self.start_position, self.goal_position, self.grid) if self.start_position != self.goal_position else []
        self.current_path_index = 0
        self.path_lock = threading.Lock()
        self.sensor_data_lock = threading.Lock()
        self.fused_sensor_data = None
        self.keep_running = True
        self.obstacle_thread = threading.Thread(target=self.continual_obstacle_check)
        self.path_thread = threading.Thread(target=self.continual_path_update)
        self.obstacle_thread.start()
        self.path_thread.start()
        self.steering_threshold = 10

    def continual_obstacle_check(self):
        while self.keep_running:
            sensor_data = self.obstacle_checker.check_for_obstacles()
            with self.sensor_data_lock:
                self.fused_sensor_data = sensor_data
            time.sleep(0.5)

    def continual_path_update(self):
        while self.keep_running:
            if self.fused_sensor_data is not None:
                current_position = self.gps_to_grid(*get_current_gps())
                if self.should_update_path(current_position):
                    with self.path_lock:
                        self.planned_path = a_star(current_position, self.goal_position, self.grid)
            time.sleep(1)

    def should_update_path(self, current_position):
    # Check if there's an obstacle closer than the max allowed distance
        if self.fused_sensor_data is not None:
            obstacle_close = np.any(self.fused_sensor_data < self.obstacle_checker.config['max_distance'])
            if obstacle_close:
                return True
        
        # Check if the robot has significantly deviated from the planned path
        if not self.planned_path:
            return False
        # Computing the minimum distance to the path
        path_distances = np.linalg.norm(np.array(self.planned_path) - np.array(current_position), axis=1)
        closest_distance = np.min(path_distances)
        if closest_distance > 10:  # Threshold to decide if we need to recalculate the path
            return True
        
        return False


    def main_loop(self):
        try:
            while not self.reached_goal():
                current_heading = get_current_heading()
                if np.any(self.fused_sensor_data):  # Modified this line
                    with self.sensor_data_lock:
                        histogram = self.vfh.compute_histogram(self.fused_sensor_data)
                        steering_direction = self.vfh.find_safe_direction(histogram, current_heading)
                else:
                    steering_direction = self.calculate_path_direction()

                self.move_robot(steering_direction)
                time.sleep(1)
        finally:
            self.keep_running = False
            self.obstacle_thread.join()
            self.path_thread.join()
            self.robot.stop()
            self.lidar_scanner.close()


    def initialize_grid(self, current_loc, goal_loc, expected_range_m, base_resolution):
        actual_distance_m = np.linalg.norm(np.array(current_loc) - np.array(goal_loc)) * 111000
        actual_distance_m = max(actual_distance_m, 0.1)
        scale = base_resolution / actual_distance_m
        origin = calculate_midpoint(current_loc, goal_loc)
        grid_resolution = int(base_resolution * (expected_range_m / max(actual_distance_m, 10)))
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

    def calculate_path_direction(self):
        if self.planned_path and self.current_path_index < len(self.planned_path):
            current_position = self.gps_to_grid(*get_current_gps())
            next_position = self.planned_path[self.current_path_index]
            if next_position == current_position:
                self.current_path_index += 1
                if self.current_path_index < len(self.planned_path):
                    next_position = self.planned_path[self.current_path_index]
                else:
                    return None
            direction_angle = np.degrees(np.arctan2(next_position[1] - current_position[1], next_position[0] - current_position[0]))
            return int((direction_angle + 360) % 360)
        return None

    def reached_goal(self):
        current_position = get_current_gps()
        goal_reached = np.linalg.norm(np.array(current_position) - np.array(self.goal_position)) < 0.0001
        return goal_reached

    def move_robot(self, steering_direction):
        if steering_direction is None:
            return
        current_heading = get_current_heading()
        error = steering_direction - current_heading
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360
        if abs(error) < self.steering_threshold:
            self.robot.forward(100)
        elif error > 0:
            turn_angle = min(self.max_turn_angle, error)
            self.robot.turn_right(70, 1 - turn_angle / 100)
        else:
            turn_angle = min(self.max_turn_angle, -error)
            self.robot.turn_left(70, 1 - turn_angle / 100)

if __name__ == "__main__":
    robot_controller = RobotController(config)
    robot_controller.main_loop()
