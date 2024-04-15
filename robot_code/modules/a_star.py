import numpy as np
import heapq
import logging

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s: %(message)s')

def manhattan_distance(a, b):
    """Calculate the Manhattan distance between two points."""
    return abs(b[0] - a[0]) + abs(b[1] - a[1])

def euclidean_distance(a, b):
    """Calculate the Euclidean distance between two points."""
    return np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

class PriorityQueue:
    """A priority queue that supports item re-prioritization."""
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

'''def a_star(start, goal, grid):
    """Implements the A* pathfinding algorithm."""
    neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Only cardinal directions
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
                if neighbor in close_set:
                    continue
                if tentative_g_score < gscore.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + manhattan_distance(neighbor, goal)
                    open_set.put(neighbor, fscore[neighbor])
    return False
'''

def a_star(start, goal, grid):
    neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Cardinal directions
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: euclidean_distance(start, goal)}
    open_set = PriorityQueue()
    open_set.put(start, fscore[start])

    while not open_set.empty():
        current = open_set.pop()
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1]  # Return reversed path

        close_set.add(current)
        for i, j in neighbors:
            neighbor = (current[0] + i, current[1] + j)
            if 0 <= neighbor[0] < grid.shape[0] and 0 <= neighbor[1] < grid.shape[1] and grid[neighbor[0]][neighbor[1]] == 0:
                tentative_g_score = gscore[current] + 1
                if neighbor in close_set:
                    continue
                if tentative_g_score < gscore.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + euclidean_distance(neighbor, goal)
                    open_set.put(neighbor, fscore[neighbor])
    return False