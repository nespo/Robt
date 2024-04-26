import numpy as np
import heapq
import utm

class NavigationSystem:
    def __init__(self, grid_resolution=0.5, grid_size=2000):
        self.grid_resolution = grid_resolution
        self.grid_size = grid_size
        self.grid = np.zeros((grid_size, grid_size), dtype=np.uint8)
        self.current_utm = None
        self.grid_origin = None

    def update_current_position(self, lat, lon):
        self.current_utm = self.gps_to_utm(lat, lon)
        self.grid_origin = (self.current_utm[0] - 500, self.current_utm[1] - 500)
    
    def gps_to_utm(self, lat, lon):
        utm_conversion = utm.from_latlon(lat, lon)
        return utm_conversion[0], utm_conversion[1]

    def utm_to_grid(self, x, y):
        grid_x = int((x - self.grid_origin[0]) / self.grid_resolution)
        grid_y = int((y - self.grid_origin[1]) / self.grid_resolution)
        return grid_x, grid_y

    def heuristic(self, a, b):
        return np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

    def update_obstacles(self, sensor_data):
        """Update the grid based on sensor data, interpreting each reading as an obstacle."""
        for angle, distance in sensor_data:
            if distance == 0:  # Ignore erroneous zero distance readings
                continue
            # Convert polar to Cartesian coordinates (relative to current UTM position)
            angle_rad = np.radians(angle)
            x_obstacle = self.current_utm[0] + distance * np.cos(angle_rad)
            y_obstacle = self.current_utm[1] + distance * np.sin(angle_rad)
            # Convert UTM coordinates of obstacle to grid coordinates
            grid_x, grid_y = self.utm_to_grid(x_obstacle, y_obstacle)
            # Mark the grid cell as containing an obstacle if it's within grid bounds
            if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
                self.grid[grid_x, grid_y] = 1  # Marking the cell as blocked

    def plan_path(self, start_lat, start_lon, goal_lat, goal_lon):
        """Plan a path from start to goal using A* search."""
        # Update the current position
        self.update_current_position(start_lat, start_lon)
        # Convert start and goal from lat/lon to UTM
        start_utm = self.gps_to_utm(start_lat, start_lon)
        goal_utm = self.gps_to_utm(goal_lat, goal_lon)
        # Convert UTM coordinates to grid coordinates
        start_grid = self.utm_to_grid(*start_utm)
        goal_grid = self.utm_to_grid(*goal_utm)
        # Perform A* search to find the path on the current grid
        path = self.a_star_search(start_grid, goal_grid, self.grid)
        return path


    # You need to ensure that the a_star_search method is correctly part of the class
    def a_star_search(self, start, goal, grid):
        """A* search to find the shortest path from start to goal avoiding obstacles."""
        # Code for A* search...
        # You might need to pass 'self' to 'heuristic' if it's not a static method:
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
                return path[::-1]  # Return reversed path
            
            close_set.add(current)
            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j
                if 0 <= neighbor[0] < grid.shape[0] and 0 <= neighbor[1] < grid.shape[1]:
                    if grid[neighbor[0], neighbor[1]] == 1:  # Check if the cell is blocked
                        continue
                    tentative_g_score = gscore[current] + self.heuristic(current, neighbor)
                    if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                        continue
                    if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                        came_from[neighbor] = current
                        gscore[neighbor] = tentative_g_score
                        fscore[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                        heapq.heappush(oheap, (fscore[neighbor], neighbor))
        return False  # Return False if no path is found

# Function to simulate receiving data from RPLIDAR A1M8
def simulate_rplidar_data():
    return [(angle, np.random.randint(1, 1000)) for angle in range(360)]

# VFH+ algorithm class
class VFHPlus:
    def __init__(self, robot_size, sector_angle, threshold):
        self.robot_size = robot_size  # Size of the robot to calculate safety distances
        self.sector_angle = sector_angle  # Angle width of histogram sectors
        self.threshold = threshold  # Threshold for treating sectors as non-navigable
    
    def update_histogram(self, sensor_data):
        num_sectors = int(360 / self.sector_angle)
        histogram = np.zeros(num_sectors)
        for angle, distance in sensor_data:
            if distance > 0:  # Only consider positive distances
                sector = int(angle / self.sector_angle) % num_sectors
                histogram[sector] += 1 / distance  # Simple inverse distance weighting
        return histogram

    def find_best_direction(self, histogram):
        # Find sector with the minimum value above threshold
        navigable = [i for i, val in enumerate(histogram) if val < self.threshold]
        if not navigable:
            return None  # No navigable path found
        # Choose the sector closest to the forward direction (sector 0)
        best_sector = min(navigable, key=lambda x: abs(x - len(histogram)//2))
        best_angle = best_sector * self.sector_angle
        return best_angle

# Main loop for dynamic navigation
def dynamic_navigation(nav_system, start_lat, start_lon, goal_lat, goal_lon):
    nav_system.update_current_position(start_lat, start_lon)
    start_utm = nav_system.current_utm
    goal_utm = nav_system.gps_to_utm(goal_lat, goal_lon)
    start_grid = nav_system.utm_to_grid(*start_utm)
    goal_grid = nav_system.utm_to_grid(*goal_utm)
    
    # Pass the grid stored in the NavigationSystem instance
    global_path = nav_system.a_star_search(start_grid, goal_grid, nav_system.grid)
    vfh = VFHPlus(robot_size=50, sector_angle=10, threshold=0.2)
    
    for step in global_path:
        sensor_data = simulate_rplidar_data()
        nav_system.update_obstacles(sensor_data)
        histogram = vfh.update_histogram(sensor_data)
        direction = vfh.find_best_direction(histogram)
        if direction is not None:
            # Convert direction to UTM and move the robot
            print("Move to direction:", direction)
        else:
            print("No path found, recalculating...")
            return dynamic_navigation(nav_system, *nav_system.gps_to_utm(*step), *goal_utm)


nav_system = NavigationSystem()
dynamic_navigation(nav_system, 62.878815, 27.637536, 62.880338, 27.635195)
