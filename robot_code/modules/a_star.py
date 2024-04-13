import numpy as np
import heapq

def heuristic(a, b):
    """Calculate the Euclidean distance between two points."""
    return np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

def latlon_to_grid(latitude, longitude, origin_lat, origin_lon, scale=1):
    """Convert latitude and longitude to grid coordinates."""
    x = int((longitude - origin_lon) * scale)
    y = int((latitude - origin_lat) * scale)
    return (y, x)

def gps_to_grid(gps_data, origin, scale):
    """Convert GPS data dictionary to grid coordinates."""
    lat = gps_data['latitude']
    lon = gps_data['longitude']
    return latlon_to_grid(lat, lon, origin[0], origin[1], scale)


def a_star(start, goal, grid):
    neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)]
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
    oheap = []

    heapq.heappush(oheap, (fscore[start], start))
    
    while oheap:
        current = heapq.heappop(oheap)[1]

        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data[::-1]

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            if 0 <= neighbor[0] < grid.shape[0] and 0 <= neighbor[1] < grid.shape[1] and grid[neighbor[0]][neighbor[1]] == 0:
                tentative_g_score = gscore[current] + heuristic(current, neighbor)
                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                    continue
                if tentative_g_score < gscore.get(neighbor, float('inf')) or neighbor not in [i[1]for i in oheap]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))
    return False
