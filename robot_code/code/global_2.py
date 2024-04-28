import numpy as np
import heapq
import utm
import os
import sys
import time
import math
import threading
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
            raise ValueError("GPS signal lost")
        try:
            self.current_utm = self.gps_to_utm(lat, lon)
            self.grid_origin = (self.current_utm[0] - 500, self.current_utm[1] - 500)
            #print(f"GPS coordinates received: Latitude = {lat}, Longitude = {lon}")
            #print(f"Converted to UTM coordinates: Easting = {self.current_utm[0]}, Northing = {self.current_utm[1]}")
            #print("Updated current position to:", self.current_utm)
        except Exception as e:
            raise RuntimeError(f"Invalid UTM conversion: {e}")

    def gps_to_utm(self, lat, lon):
        utm_conversion = utm.from_latlon(lat, lon)
        return utm_conversion[0], utm_conversion[1]

    def utm_to_grid(self, x, y):
        grid_x = int((x - self.grid_origin[0]) / self.grid_resolution)
        grid_y = int((y - self.grid_origin[1]) / self.grid_resolution)
        return grid_x, grid_y

    def heuristic(self, a, b):
        return np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

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
                print("Path found!")
                return self.reconstruct_path(came_from, current)
            close_set.add(current)
            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j
                if not self.is_valid_position(grid, neighbor, close_set):
                    continue
                tentative_g_score = gscore[current] + self.heuristic(current, neighbor)
                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                    continue
                if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))
                    #print(f"New path considered to: {neighbor} with total cost: {fscore[neighbor]}")
        print("No path found.")
        return False

    def is_valid_position(self, grid, pos, close_set):
        if 0 <= pos[0] < grid.shape[0] and 0 <= pos[1] < grid.shape[1]:
            if grid[pos[0], pos[1]] == 1:  # Assumes 1 is an obstacle
                return False
            return True
        return False

    def reconstruct_path(self, came_from, current):
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.append(current)
        print(f"Reconstructing path: {path}")
        return path[::-1]

    def dynamic_update_position(self):
        while self.running:
            lat, lon = get_current_gps()
            self.update_current_position(lat, lon)
            time.sleep(1)  # Update every second


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

def dynamic_navigation(nav_system, start_lat, start_lon, goal_lat, goal_lon, robot):
    print("Dynamic navigation started.")
    nav_system.update_current_position(start_lat, start_lon)
    start_utm = nav_system.current_utm
    goal_utm = nav_system.gps_to_utm(goal_lat, goal_lon)
    start_grid = nav_system.utm_to_grid(*start_utm)
    goal_grid = nav_system.utm_to_grid(*goal_utm)
    print(f"New A* search called from grid position: {start_grid} to {goal_grid}")
    global_path = nav_system.a_star_search(start_grid, goal_grid, nav_system.grid)
    plot_environment(nav_system, goal_utm) 
    if not global_path:
        print("No global path could be planned.")
        robot.stop()
        return

    path_in_utm = [(nav_system.grid_origin[0] + x * nav_system.grid_resolution, nav_system.grid_origin[1] + y * nav_system.grid_resolution) for x, y in global_path]
    for i in range(len(path_in_utm) - 1):
        current_position = path_in_utm[i]
        next_position = path_in_utm[i+1]
        current_heading = get_current_heading()
        required_bearing = calculate_bearing(current_position, next_position)
        required_turn = required_bearing - current_heading
        target_heading = (current_heading + required_turn) % 360
        if required_turn != 0:
            turn_power = max(0, min(100, abs(required_turn)))
            if required_turn < 0:
                robot.turn_left(turn_power)
            else:
                robot.turn_right(turn_power)
            print(f"Turning towards new heading: {target_heading} with power {turn_power}")
        wait_until_turn_complete(robot, target_heading)
        speed = min(50, 100)  # Define a mechanism to determine speed if necessary
        robot.forward(speed)
        print(f"Moving forward at speed: {speed}")
    print("Navigation complete.")

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

def plot_environment(nav_system, goal_utm):
    fig, ax = plt.subplots()
    ax.set_xlim(0, nav_system.grid_size)
    ax.set_ylim(0, nav_system.grid_size)
    start_grid = nav_system.utm_to_grid(*nav_system.current_utm)
    goal_grid = nav_system.utm_to_grid(*goal_utm)

    # Calculate and plot the initial path using A* search
    path = nav_system.a_star_search(start_grid, goal_grid, nav_system.grid)
    if path:
        path_x, path_y = zip(*path)
        ax.plot(path_x, path_y, '-o', color='blue', label='Path')  # Plot the path as a line with markers
        scat_start = ax.scatter(*start_grid, color='green', marker='o', s=100, label='Start')  # Start point
        scat_goal = ax.scatter(*goal_grid, color='red', marker='x', s=100, label='Goal')  # Goal point
        ax.legend()

        # Real-time position marker, initialized at start position
        scat_current_pos, = ax.plot(*start_grid, 'bo', ms=10, label='Current Position')

        def update(frame):
            # Update the current position from the navigation system
            current_grid = nav_system.utm_to_grid(*nav_system.current_utm)
            scat_current_pos.set_data(current_grid[0], current_grid[1])
            print(f"Updating plot with current position: {current_grid}")
            return scat_current_pos,

        # Create an animation that updates the robot's position dynamically
        ani = FuncAnimation(fig, update, blit=True, interval=1000)

        plt.show()

    else:
        print("No path or plot to display.")
        print("No path or plot to display.")


def main():
    nav_system = NavigationSystem()
    start_lat, start_lon = get_current_gps()
    goal_lat, goal_lon = 62.880338, 27.635195  # Static goal location
    goal_utm = nav_system.gps_to_utm(goal_lat, goal_lon)
    robot = Robot(config)
    
    # Start dynamic position update thread
    nav_system.running = True
    position_thread = threading.Thread(target=nav_system.dynamic_update_position)
    position_thread.start()
    
    # Start dynamic navigation
    dynamic_navigation(nav_system, start_lat, start_lon, goal_lat, goal_lon, robot)
    plot_environment(nav_system, goal_utm)
    
    # Cleanup
    nav_system.running = False
    position_thread.join()

if __name__ == "__main__":
    main()
