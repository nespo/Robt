import numpy as np
import time
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

def calculate_midpoint(loc1, loc2):
    midpoint = ((loc1[0] + loc2[0]) / 2, (loc1[1] + loc2[1]) / 2)
    print(f"Calculated midpoint: {midpoint}")
    return midpoint

class VFHPlus:
    def __init__(self, cell_size=10, threshold=300, sectors=180):
        self.cell_size = cell_size
        self.threshold = threshold
        self.sectors = sectors
    print("Initialized VFH+ with finer resolution.")

    def compute_histogram(self, sensor_data):
        print("Inside histogram")
        histogram = np.zeros(self.sectors)
        sector_angle = 360 // self.sectors
        for angle in range(360):
            if angle >= len(sensor_data):
                continue
            sector_index = int(angle // sector_angle)  # Explicit integer conversion
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
        self.goal_loc = (62.878815, 27.637536)
        print(f"Current GPS: {self.current_loc}, Goal GPS: {self.goal_loc}")
        self.origin, self.scale, self.grid = self.initialize_grid(self.current_loc, self.goal_loc)
        self.start_position = self.gps_to_grid(self.current_loc[0], self.current_loc[1])
        self.goal_position = self.gps_to_grid(self.goal_loc[0], self.goal_loc[1])
        self.planned_path = a_star(self.start_position, self.goal_position, self.grid) if self.start_position != self.goal_position else []
        self.current_path_index = 0
        print(f"Start position on grid: {self.start_position}, Goal position on grid: {self.goal_position}")


    def calculate_distance(loc1, loc2):
        lat_diff = (loc1[0] - loc2[0]) * 111319.9  # convert degrees to meters
        lon_diff = (loc1[1] - loc2[1]) * 111319.9  # convert degrees to meters
        return np.sqrt(lat_diff**2 + lon_diff**2)

    def initialize_grid(self, current_loc, goal_loc):
        distance = self.calculate_distance(current_loc, goal_loc)
        print(f"Distance: {distance}")
        min_cells_apart = 20  # Minimum cells apart for close points
        
        # Adjust scale to ensure minimum separation on the grid
        scale = 5000 / distance if distance < 5 else 500 / distance
        grid_resolution = int(max(min_cells_apart, scale * distance))

        # Ensure grid resolution is sensible
        grid_resolution = max(grid_resolution, 1000)  # Cap minimum grid size
        grid_resolution = min(grid_resolution, 5000)  # Cap maximum grid size for memory management
        grid_shape = (grid_resolution, grid_resolution)
        grid = np.zeros(grid_shape, dtype=np.float32)
        origin = calculate_midpoint(current_loc, goal_loc)
        
        return origin, scale, grid

    def gps_to_grid(self, latitude, longitude):
        x = (longitude - self.origin[1]) * self.scale
        y = (latitude - self.origin[0]) * self.scale
        grid_x = int(x) + self.grid.shape[1] // 2
        grid_y = int(y) + self.grid.shape[0] // 2
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

                # Ensure all sensor data are finite and replace 'inf' and NaN with a high but finite value
                valid_sensor_data = np.where(np.isfinite(sensor_data), sensor_data, 1000)
                print("Valid Sensor Data:", valid_sensor_data)  # Debug print

                histogram = self.vfh.compute_histogram(valid_sensor_data)
                if np.any(histogram > 0):
                    steering_direction = self.vfh.find_safe_direction(histogram, current_heading)
                    print(f"Obstacle detected, steering direction: {steering_direction}")
                else:
                    steering_direction = self.calculate_path_direction()
                    print(f"No obstacles detected, following path, steering direction: {steering_direction}")

                self.move_robot(steering_direction)
                print(f"Moving robot towards: {steering_direction}")
                time.sleep(1)  # Control loop pause for stability
                self.update_path_if_needed()
        except (KeyboardInterrupt, Exception) as e:
            print(f"An error occurred: {e}")
            self.robot.stop()
            self.lidar_scanner.close()
            print("Emergency stop! The robot and lidar scanner have been turned off.")



    def update_path_if_needed(self):
        current_position = self.gps_to_grid(*get_current_gps())
        if self.planned_path and current_position != self.planned_path[self.current_path_index]:
            print("Recalculating path due to deviation or obstacle.")
            self.planned_path = a_star(current_position, self.goal_position, self.grid)

    def reached_goal(self):
        current_position = get_current_gps()
        goal_reached = np.linalg.norm(np.array(current_position) - np.array(self.goal_position)) < 0.0001
        print(f"Current GPS position: {current_position}, Goal position: {self.goal_position}, Goal reached: {goal_reached}")
        return goal_reached

    def move_robot(self, steering_direction):
        if steering_direction is None:
            print("No valid steering direction provided. Robot will not move.")
            return  # No valid direction to move

        # Retrieve the current heading from the navigation module
        current_heading = get_current_heading()
        error = steering_direction - current_heading

        # Normalize the error to be within the range [-180, 180]
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        print(f"Steering direction: {steering_direction}, Current heading: {current_heading}, Heading error: {error}")

        # Define threshold for action
        steering_threshold = 10  # degrees within which the robot should move forward
        max_turn_angle = 50  # max degrees the robot should turn at once

        # Determine movement based on error magnitude and direction
        if abs(error) < steering_threshold:
            print("Moving forward")
            self.robot.forward(100)  # Move forward with power level 50
        elif error > 0:
            turn_angle = min(max_turn_angle, error)  # Limit the turn to max_turn_angle
            print(f"Turning right: {turn_angle} degrees")
            self.robot.turn_right(70, 1 - turn_angle / 100)  # Reduce inside wheel speed based on turn angle
        else:
            turn_angle = min(max_turn_angle, -error)  # Limit the turn to max_turn_angle
            print(f"Turning left: {turn_angle} degrees")
            self.robot.turn_left(70, 1 - turn_angle / 100)  # Reduce inside wheel speed based on turn angle


    def reverse_and_reroute(self):
        print("Obstacle encountered, reversing and rerouting.")
        self.robot.backward(50)
        time.sleep(2)  # Reverse for 2 seconds
        self.update_path_if_needed()

    def halt_and_reassess(self):
        print("Halt and reassessing situation.")
        self.robot.stop()
        time.sleep(5)  # Wait and reassess
        self.update_path_if_needed()

    def emergency_stop(self):
        print("Emergency stop triggered.")
        self.robot.stop()


if __name__ == "__main__":
    robot_controller = RobotController(config)
    robot_controller.main_loop()
