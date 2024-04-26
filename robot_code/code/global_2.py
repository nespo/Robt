import numpy as np
import heapq
import utm
import os
import sys

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

    def plan_path(self, start_lat, start_lon, goal_lat, goal_lon):
        print("Planning path from start to goal.")
        self.update_current_position(start_lat, start_lon)
        start_utm = self.gps_to_utm(start_lat, start_lon)
        goal_utm = self.gps_to_utm(goal_lat, goal_lon)
        start_grid = self.utm_to_grid(*start_utm)
        goal_grid = self.utm_to_grid(*goal_utm)
        path = self.a_star_search(start_grid, goal_grid, self.grid)
        return path

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
    print(sensor_data)
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
        return histogram

    def find_best_direction(self, histogram):
        navigable = [i for i, val in enumerate(histogram) if val < self.threshold]
        if not navigable:
            print("No navigable path found.")
            return None
        best_sector = min(navigable, key=lambda x: abs(x - len(histogram)//2))
        best_angle = best_sector * self.sector_angle
        print("Best direction found:", best_angle)
        return best_angle

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
    
    current_heading = get_current_heading()
    print("Current heading:", current_heading)
    
    for step in global_path:
        #sensor_data = rplidar_data()
        sensor_data = ultrasonic_data()
        nav_system.update_obstacles(sensor_data)
        histogram = vfh.update_histogram(sensor_data)
        direction = vfh.find_best_direction(histogram)
        if direction is not None:
            required_turn = direction - current_heading
            print(f"reuired turn: {required_turn}")
            if required_turn < 0:
                robot.turn_left(abs(required_turn))
            elif required_turn > 0:
                robot.turn_right(abs(required_turn))
            robot.forward(50)
        else:
            print("Obstacle detected, recalculating path...")
            dynamic_navigation(nav_system, *nav_system.gps_to_utm(*step), *goal_utm)

nav_system = NavigationSystem()
start_lat, start_lon = get_current_gps()
robot = Robot(config)
dynamic_navigation(nav_system, start_lat, start_lon, 62.880338, 27.635195, robot)
