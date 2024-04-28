import numpy as np
import heapq
import utm
import os
import sys
import threading
import time
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Correct library import paths
script_dir = os.path.dirname(os.path.realpath(__file__))
parent_dir = os.path.join(script_dir, '..', '..')
sys.path.append(os.path.abspath(parent_dir))

# Import robot-specific modules
from robot_code.code.motor_control import Robot
from robot_code.code.config import config
from robot_code.modules.nav import get_current_gps, get_current_heading

class NavigationSystem:
    def __init__(self, grid_resolution=0.5, grid_size=2000):
        self.grid_resolution = grid_resolution
        self.grid_size = grid_size
        self.grid = np.zeros((grid_size, grid_size), dtype=np.uint8)
        self.current_utm = None
        self.grid_origin = None
        print("Navigation System initialized with grid size:", grid_size, "and resolution:", grid_resolution)

    def update_current_position(self, lat, lon):
        if lat is None or lon is None:
            print("Error: GPS signal lost")
            return False
        try:
            self.current_utm = self.gps_to_utm(lat, lon)
            self.grid_origin = (self.current_utm[0] - 500, self.current_utm[1] - 500)
            print(f"GPS Data Retrieved: Latitude = {lat}, Longitude = {lon}")
            return True
        except Exception as e:
            print(f"Error in UTM conversion: {e}")
            return False


    def gps_to_utm(self, lat, lon):
        utm_conversion = utm.from_latlon(lat, lon)
        return utm_conversion[0], utm_conversion[1]

    def utm_to_grid(self, x, y):
        grid_x = int((x - self.grid_origin[0]) / self.grid_resolution)
        grid_y = int((y - self.grid_origin[1]) / self.grid_resolution)
        return grid_x, grid_y

    def a_star_search(self, start, goal):
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
                path = self.reconstruct_path(came_from, current)
                print("Path found!")
                return path
            close_set.add(current)
            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j
                if not self.is_valid_position(neighbor, close_set):
                    continue
                tentative_g_score = gscore[current] + self.heuristic(current, neighbor)
                if tentative_g_score < gscore.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))
        print("No path found.")
        return None


    def is_valid_position(self, pos, close_set):
        x, y = pos
        if 0 <= x < self.grid_size and 0 <= y < self.grid_size:
            return self.grid[x, y] == 0 and pos not in close_set
        return False

    def heuristic(self, a, b):
        return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    def reconstruct_path(self, came_from, current):
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.append(current)
        return path[::-1]


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0

    def update(self, error, delta_time):
        self.integral += error * delta_time
        derivative = (error - self.previous_error) / delta_time
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output


def calculate_bearing(pointA, pointB):
    lat1, lon1 = map(math.radians, pointA)
    lat2, lon2 = map(math.radians, pointB)
    diffLong = lon2 - lon1
    x = math.sin(diffLong) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(diffLong))
    initial_bearing = math.atan2(x, y)
    initial_bearing = math.degrees(initial_bearing)
    compass_bearing = (initial_bearing + 360) % 360
    return compass_bearing

def wait_until_turn_complete(robot, target_heading, tolerance=20):
    pid = PIDController(kp=0.5, ki=0.1, kd=0.05)
    current_heading = get_current_heading()
    while abs(current_heading - target_heading) > tolerance:
        error = target_heading - current_heading
        power = max(10, min(100, abs(pid.update(error, 0.1))))
        if error < 0:
            robot.turn_left(power)
        else:
            robot.turn_right(power)
        time.sleep(5)
        current_heading = get_current_heading()
        print(f"Adjusting heading: Current: {current_heading}, Target: {target_heading}, error we have: {error}")
    print("Turn complete. Current heading:", current_heading)
    robot.stop()


def dynamic_navigation(nav_system, robot):
    try:
        start_lat, start_lon = get_current_gps()
        goal_lat, goal_lon = 62.880338, 27.635195
        if nav_system.update_current_position(start_lat, start_lon):
            start_utm = nav_system.current_utm
            goal_utm = nav_system.gps_to_utm(goal_lat, goal_lon)
            start_grid = nav_system.utm_to_grid(*start_utm)
            goal_grid = nav_system.utm_to_grid(*goal_utm)
            path = nav_system.a_star_search(start_grid, goal_grid)
            if path:
                follow_path(nav_system, robot, path)
            else:
                print("No global path could be planned.")
                robot.stop()
    except Exception as e:
        print(f"Error during navigation: {e}")
        robot.stop()

def follow_path(nav_system, robot, path):
    path_in_utm = [(nav_system.grid_origin[0] + x * nav_system.grid_resolution, nav_system.grid_origin[1] + y * nav_system.grid_resolution) for x, y in path]
    for i in range(len(path_in_utm) - 1):
        current_position = path_in_utm[i]
        next_position = path_in_utm[i+1]
        current_heading = get_current_heading()
        required_bearing = calculate_bearing(current_position, next_position)
        required_turn = required_bearing - current_heading
        target_heading = (current_heading + required_turn) % 360

        if abs(required_turn) > 20:  # Check if the turn is needed based on tolerance
            wait_until_turn_complete(robot, target_heading, 20)  # Wait for turn to complete within the specified tolerance

        robot.forward(50)  # Define speed based on operational conditions
        print(f"Moving from {current_position} to {next_position} at speed 50")

def main():
    nav_system = NavigationSystem()
    robot = Robot(config)
    dynamic_navigation(nav_system, robot)

if __name__ == "__main__":
    main()