import numpy as np
import heapq
import utm

def get_current_gps():
    # Mock GPS data; replace with real data retrieval
    return 62.878815, 27.637536 # Example: Eiffel Tower coordinates

def get_current_heading():
    # Mock heading data; replace with real data retrieval
    return 90  # Example: facing east (in degrees)

def gps_to_utm(lat, lon):
    # Convert GPS to UTM coordinates
    utm_conversion = utm.from_latlon(lat, lon)
    return utm_conversion[0], utm_conversion[1]  # Returns easting, northing, zone_number, zone_letter

def utm_to_grid(x, y, grid_resolution, grid_origin):
    # Convert UTM coordinates to grid coordinates based on resolution and origin
    grid_x = int((x - grid_origin[0]) / grid_resolution)
    grid_y = int((y - grid_origin[1]) / grid_resolution)
    return grid_x, grid_y

def heuristic(a, b):
    # Use Euclidean distance as heuristic
    return np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

def a_star_search(start, goal, grid):
    # Implement A* search algorithm
    neighbors = [(0,1), (1,0), (0,-1), (-1,0), (1,1), (-1,-1), (1,-1), (-1,1)]  # 8-connectivity
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
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
            return path[::-1]

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < grid.shape[0] and 0 <= neighbor[1] < grid.shape[1]:
                if grid[neighbor[0]][neighbor[1]] == 1:  # Check if the grid cell is blocked
                    continue

                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                    continue

                if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))
    return False

# Simulation parameters
grid_resolution = 0.5  # 0.5 m per grid cell
grid_size = 2000  # Grid dimensions (1000x1000 meters adjusted for 0.5 m resolution)
current_gps = get_current_gps()
current_utm = gps_to_utm(*current_gps)
grid_origin = (current_utm[0] - 500, current_utm[1] - 500)  # Center the origin around the current location

# Convert goal GPS coordinates
goal_gps = (62.880338, 27.635195)  # Example: Palais de Tokyo
goal_utm = gps_to_utm(*goal_gps)
start = utm_to_grid(*current_utm, grid_resolution, grid_origin)
goal = utm_to_grid(*goal_utm, grid_resolution, grid_origin)

# Create a grid and define blocked areas (obstacles)
grid = np.zeros((grid_size, grid_size), dtype=np.uint8)
# Example obstacle: define a block around a particular area
grid[800:820, 900:920] = 1  # Adjusted obstacle size

# Find path using A*
path = a_star_search(start, goal, grid)
print("Path from start to goal:", path)
