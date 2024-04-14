import numpy as np
import heapq
import logging

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s: %(message)s')

def manhattan_distance(a, b):
    """Calculate the Manhattan distance between two points."""
    return abs(b[0] - a[0]) + abs(b[1] - a[1])

class PriorityQueue:
    """A priority queue that allows for priority updates."""
    def __init__(self):
        self.elements = {}
        self.priority_list = []

    def put(self, item, priority):
        if item in self.elements:
            self.remove(item)
        self.elements[item] = priority
        heapq.heappush(self.priority_list, (priority, item))

    def remove(self, item):
        del self.elements[item]

    def pop(self):
        while self.priority_list:
            priority, item = heapq.heappop(self.priority_list)
            if item in self.elements and self.elements[item] == priority:
                del self.elements[item]
                return item
        raise KeyError('pop from an empty priority queue')

    def empty(self):
        return not self.elements

def a_star(start, goal, grid):
    """Implements the A* pathfinding algorithm."""
    neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)]
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: manhattan_distance(start, goal)}
    open_set = PriorityQueue()
    open_set.put(start, fscore[start])

    while not open_set.empty():
        current = open_set.pop()

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1]

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            if 0 <= neighbor[0] < grid.shape[0] and 0 <= neighbor[1] < grid.shape[1] and grid[neighbor[0]][neighbor[1]] == 0:
                tentative_g_score = gscore[current] + 1
                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, float('inf')):
                    continue
                if tentative_g_score < gscore.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + manhattan_distance(neighbor, goal)
                    open_set.put(neighbor, fscore[neighbor])
    return False
'''
class NavigationSystem:
    def __init__(self, origin_lat, origin_lon, scale, grid_size=2000):
        self.origin = (origin_lat, origin_lon)
        self.scale = scale
        self.grid = np.zeros((grid_size, grid_size))  # Dynamic grid size
        logging.info("Navigation system initialized with grid size %dx%d", grid_size, grid_size)

    def gps_to_grid(self, latitude, longitude):
        x = int((longitude - self.origin[1]) * self.scale)
        y = int((latitude - self.origin[0]) * self.scale)
        if 0 <= x < self.grid.shape[1] and 0 <= y < self.grid.shape[0]:
            return (y, x)
        else:
            logging.error("GPS coordinates out of grid bounds: (%f, %f)", latitude, longitude)
            raise ValueError("GPS coordinates out of grid bounds")

    def plan_path(self, current_lat, current_lon, goal_lat, goal_lon):
        start = self.gps_to_grid(current_lat, current_lon)
        goal = self.gps_to_grid(goal_lat, goal_lon)
        path = a_star(start, goal, self.grid)
        if path:
            logging.info("Path found with length %d", len(path))
        else:
            logging.warning("No path found")
        return path

# Example usage
nav_system = NavigationSystem(62.8789185, 27.6376086, 221744, 2000)
current_position = (62.8789185, 27.6376086)  # Dynamic current position
goal_position = (62.879918, 27.638608)  # Example goal, approximately 500 meters away

path = nav_system.plan_path(*current_position, *goal_position)
print("Planned Path:", path if path else "No path available")'''
