import numpy as np
import threading
import time
import logging
from collections import deque
import os, sys


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

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s: %(message)s')

def euclidean_distance(a, b):
    return np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

def angular_difference(target, current):
    diff = target - current
    diff = (diff + 180) % 360 - 180
    return diff

class AStarPlanner:
    def __init__(self, grid):
        self.grid = grid

    def heuristic(self, a, b):
        return euclidean_distance(a, b)

    def a_star(self, start, goal):
        neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        close_set = set()
        came_from = {}
        gscore = {start: 0}
        fscore = {start: self.heuristic(start, goal)}
        open_set = deque([start])

        while open_set:
            current = min(open_set, key=lambda x: fscore[x])
            if current == goal:
                return self.reconstruct_path(came_from, current)

            open_set.remove(current)
            close_set.add(current)

            for dx, dy in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)
                if 0 <= neighbor[0] < self.grid.shape[0] and 0 <= neighbor[1] < self.grid.shape[1] and self.grid[neighbor[0]][neighbor[1]] == 0:
                    if neighbor in close_set:
                        continue
                    tentative_g_score = gscore[current] + 1
                    if neighbor not in open_set or tentative_g_score < gscore.get(neighbor, float('inf')):
                        came_from[neighbor] = current
                        gscore[neighbor] = tentative_g_score
                        fscore[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                        if neighbor not in open_set:
                            open_set.append(neighbor)
        return []

    def reconstruct_path(self, came_from, current):
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.append(current)
        return path[::-1]

class NavigationSystem:
    def __init__(self, destination, config, grid_size=(500, 500), resolution=1):
        self.robot = Robot(config)
        self.lidar = LidarScanner('/dev/ttyUSB0')
        self.obstacle_checker = ObstacleChecker(self.lidar, Ultrasonic(Pin('D8'), Pin('D9')), {'max_distance': 4000})
        self.destination = destination
        self.grid = np.zeros(grid_size)
        self.planner = AStarPlanner(self.grid)
        self.path = deque()
        self.resolution = resolution
        self.update_grid_scale(grid_size, resolution)
        self.current_position = get_current_gps()
        self.current_heading = get_current_heading()

    def update_grid_scale(self, size, resolution):
        # Scale grid size based on expected travel distance
        dimension = max(size)
        scale_factor = max(1, int((500 / resolution) / dimension))
        self.grid = np.zeros((dimension * scale_factor, dimension * scale_factor))

    def update_position_and_plan_path(self):
        self.current_position = get_current_gps()
        self.current_heading = get_current_heading()
        if not self.path or self.obstacle_checker.check_for_obstacles():
            start = (int(self.current_position[0] * self.resolution), int(self.current_position[1] * self.resolution))
            goal = (int(self.destination[0] * self.resolution), int(self.destination[1] * self.resolution))
            self.path = deque(self.planner.a_star(start, goal))

    def navigate(self):
        if self.path:
            next_position = self.path.popleft()
            while next_position != tuple(self.current_position):
                target_heading = np.degrees(np.arctan2(next_position[1] - self.current_position[1],
                                                      next_position[0] - self.current_position[0]))
                required_turn = angular_difference(target_heading, self.current_heading)
                if abs(required_turn) > 10:
                    if required_turn > 0:
                        self.robot.turn_right(abs(required_turn))
                    else:
                        self.robot.turn_left(abs(required_turn))
                self.robot.forward(50)  # Adjust speed as necessary
                time.sleep(1)  # Adjust timing based on real-world testing
                self.current_position = get_current_gps()
                self.current_heading = get_current_heading()

            if not self.path:
                logging.info("Destination reached.")
                self.robot.stop()

    def run(self):
        try:
            while True:
                self.update_position_and_plan_path()
                self.navigate()
                time.sleep(0.5)  # Adjust loop frequency as necessary
        except KeyboardInterrupt:
            self.robot.stop()
            logging.info("Navigation stopped by user.")

if __name__ == "__main__":
    destination = (62.878815, 27.637536)  # Destination GPS coordinates
    navigation_system = NavigationSystem(destination, config)
    navigation_thread = threading.Thread(target=navigation_system.run)
    navigation_thread.start()
