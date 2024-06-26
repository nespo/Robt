import numpy as np
import heapq
import utm
import os
import sys
import time

# Correct library import paths
script_dir = os.path.dirname(os.path.realpath(__file__))
parent_dir = os.path.join(script_dir, '..', '..')
sys.path.append(os.path.abspath(parent_dir))

# Import robot-specific modules
from robot_code.code.motor_control import Robot
from robot_code.code.config import config
from robot_code.modules.nav import get_current_gps, get_current_heading
from robot_code.modules.ultrasonic import Ultrasonic
from robot_code.modules.pin import Pin

ultrasonic = Ultrasonic(Pin('D8'), Pin('D9'))

class NavigationSystem:
    def __init__(self, grid_resolution=0.5, grid_size=2000):
        self.grid_resolution = grid_resolution
        self.grid_size = grid_size
        self.grid = np.zeros((grid_size, grid_size), dtype=np.uint8)
        self.current_utm = None
        self.grid_origin = None
        print("Navigation System initialized with grid size:", grid_size, "and resolution:", grid_resolution)

    def update_current_position(self, lat, lon):
        self.current_utm = self.gps_to_utm(lat, lon)
        self.grid_origin = (self.current_utm[0] - 500, self.current_utm[1] - 500)
        print("Updated current position to:", self.current_utm)

    def gps_to_utm(self, lat, lon):
        utm_conversion = utm.from_latlon(lat, lon)
        return utm_conversion[0], utm_conversion[1]

    def utm_to_grid(self, x, y):
        grid_x = int((x - self.grid_origin[0]) / self.grid_resolution)
        grid_y = int((y - self.grid_origin[1]) / self.grid_resolution)
        print("Converted UTM to grid:", (x, y), "->", (grid_x, grid_y))
        return grid_x, grid_y

    def heuristic(self, a, b):
        return np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

    def update_obstacles(self, sensor_data):
        print("Updating obstacles.")
        for angle, distance in sensor_data:
            if distance == 0:
                continue
            angle_rad = np.radians(angle)
            x_obstacle = self.current_utm[0] + distance * np.cos(angle_rad)
            y_obstacle = self.current_utm[1] + distance * np.sin(angle_rad)
            grid_x, grid_y = self.utm_to_grid(x_obstacle, y_obstacle)
            if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
                self.grid[grid_x, grid_y] = 1
                print("Obstacle updated at grid position:", (grid_x, grid_y))
                return True
        return False

    def a_star_search(self, start, goal, grid):
        print("Starting A* search from", start, "to", goal)
        neighbors = [(0,1), (1,0), (0,-1), (-1,0), (1,1), (-1,-1), (1,-1), (-1,1)]
        close_set = set()
        came_from = {}
        gscore = {start: 0}
        fscore = {start: self.heuristic(start, goal)}
        oheap = []

        heapq.heappush(oheap, (fscore[start], start))

        while oheap:
            current = heapq.heappop(oheap)[1]
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(current)
                print("Path found:", path[::-1])
                return path[::-1]

            close_set.add(current)
            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j
                if 0 <= neighbor[0] < grid.shape[0] and 0 <= neighbor[1] < grid.shape[1]:
                    if grid[neighbor[0], neighbor[1]] == 1:
                        continue
                    tentative_g_score = gscore[current] + self.heuristic(current, neighbor)
                    if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                        continue
                    if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                        came_from[neighbor] = current
                        gscore[neighbor] = tentative_g_score
                        fscore[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                        heapq.heappush(oheap, (fscore[neighbor], neighbor))
        print("No path found.")
        return False  # Return False if no path is found

def ultrasonic_data():
    sensor_data = ultrasonic.full_scan()
    print(f"Ultrasonic sensor data: {sensor_data}")
    return sensor_data

# VFH+ algorithm class
class VFHPlus:
    def __init__(self, robot_size, sector_angle, threshold):
        self.robot_size = robot_size
        self.sector_angle = sector_angle
        self.threshold = threshold
        print("VFH+ initialized with sector angle:", sector_angle, "and threshold:", threshold)

    def update_histogram(self, sensor_data):
        num_sectors = int(360 / self.sector_angle)
        histogram = np.zeros(num_sectors)
        for angle, distance in sensor_data:
            if distance > 0:
                sector = int(angle / self.sector_angle) % num_sectors
                histogram[sector] += 1 / distance
        print(f"Histogram: {histogram}")
        return histogram

    def find_best_direction(self, histogram):
        navigable = [i for i, val in enumerate(histogram) if val < self.threshold]
        if not navigable:
            print("No navigable path found.")
            return None
        best_sector = min(navigable, key=lambda x: abs(x - len(histogram)//2))
        best_angle = best_sector * self.sector_angle
        print("Best direction found from histogram:", best_angle)
        return best_angle

def wait_until_turn_complete(robot, target_heading, tolerance=5):
    """
    Blocks execution until the robot's heading is within a certain tolerance of the target heading.
    `tolerance` is the acceptable error in degrees.
    """
    current_heading = get_current_heading()
    while abs(current_heading - target_heading) > tolerance:
        error = target_heading - current_heading
        power = max(10, min(100, abs(int(error * 0.5))))  # Proportional control factor

        if error < 0:
            robot.turn_left(power)
        elif error > 0:
            robot.turn_right(power)

        time.sleep(0.1)  # Check every 100 milliseconds
        current_heading = get_current_heading()
        print(f"Adjusting heading: Current: {current_heading}, Target: {target_heading}, error we have: {error}")

    print("Turn complete. Current heading:", current_heading)
    robot.stop()  # Stop turning once the heading is achieved


#PID 

def wait_until_turn_complete(robot, target_heading, tolerance=5):
    Kp = 0.5
    Ki = 0.1
    Kd = 0.05
    previous_error = 0
    integral = 0
    current_heading = get_current_heading()
    while abs(current_heading - target_heading) > tolerance:
        error = target_heading - current_heading
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360
        integral += error * 0.1
        derivative = (error - previous_error) / 0.1
        power = Kp * error + Ki * integral + Kd * derivative
        power = max(10, min(100, abs(int(power))))
        if error < 0:
            robot.turn_left(power)
        else:
            robot.turn_right(power)
        print(f"Adjusting heading: Current = {current_heading}, Target = {target_heading}, Error = {error}")
        time.sleep(0.1)
        current_heading = get_current_heading()
        previous_error = error
    robot.stop()
    print("Turn complete. Current heading:", current_heading)
# Main loop for dynamic navigation
def dynamic_navigation(nav_system, start_lat, start_lon, goal_lat, goal_lon, robot):
    print("Dynamic navigation started.")
    nav_system.update_current_position(start_lat, start_lon)
    start_utm = nav_system.current_utm
    goal_utm = nav_system.gps_to_utm(goal_lat, goal_lon)
    start_grid = nav_system.utm_to_grid(*start_utm)
    goal_grid = nav_system.utm_to_grid(*goal_utm)
    
    global_path = nav_system.a_star_search(start_grid, goal_grid, nav_system.grid)
    if not global_path:
        print("No global path could be planned.")
        robot.stop()
        return

    vfh = VFHPlus(robot_size=20, sector_angle=15, threshold=0.3)
    
    # Adjust the robot's heading based on the best navigable direction found
    current_heading = get_current_gps()[1]
    print("Current heading:", current_heading)

    for step in global_path:
        sensor_data = ultrasonic_data()
        if nav_system.update_obstacles(sensor_data):
            print("Obstacle detected, recalculating path...")
            dynamic_navigation(nav_system, get_current_gps()[0], get_current_gps()[1], goal_lat, goal_lon, robot)
            return  # Important to return to avoid further execution after recursion

        histogram = vfh.update_histogram(sensor_data)
        direction = vfh.find_best_direction(histogram)
        if direction is not None:
            required_turn = direction - current_heading
            target_heading = (current_heading + required_turn) % 360  # Ensure the heading wraps around correctly
            print(f"Required turn: {required_turn}, Target heading: {target_heading}")

            if required_turn < 0:
                turn_power = max(0, min(100, abs(required_turn)))
                robot.turn_left(turn_power)
            elif required_turn > 0:
                turn_power = max(0, min(100, abs(required_turn)))
                robot.turn_right(turn_power)

            # Wait for the turn to complete
            wait_until_turn_complete(robot, target_heading)

            # Move forward after the turn is complete
            robot.forward(min(50, 100))  # Example forward power
        else:
            print("No direction found, stopping robot.")
            robot.stop()
            break

# Initial setup
nav_system = NavigationSystem()
start_lat, start_lon = get_current_gps()
robot = Robot(config)
dynamic_navigation(nav_system, start_lat, start_lon, 62.880338, 27.635195, robot)

