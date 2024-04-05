import numpy as np
import json
import time
import os

class OccupancyGrid:
    def __init__(self, size=300, resolution=1):
        self.grid = np.zeros((size, size))
        self.resolution = resolution  # cm per grid cell

    def update(self, angle, distance):
        # Assuming the sensor is at the center of the grid
        x = int(distance * np.cos(np.radians(angle)) + self.grid.shape[0] / 2)
        y = int(distance * np.sin(np.radians(angle)) + self.grid.shape[1] / 2)
        if 0 <= x < self.grid.shape[0] and 0 <= y < self.grid.shape[1]:
            self.grid[x][y] = 1  # Mark as occupied

    def save(self, filename="map.json"):
        with open(filename, 'w') as outfile:
            json.dump(self.grid.tolist(), outfile)

    def load(self, filename="map.json"):
        if os.path.exists(filename):
            with open(filename, 'r') as infile:
                self.grid = np.array(json.load(infile))
            return True
        return False

def scan_and_update_map(ultrasonic, servo, occupancy_grid, angle_range=180, step=18):
    for angle in range(-angle_range // 2, angle_range // 2 + 1, step):
        servo_angle = 90 + angle  # Assuming 90 is forward
        servo.set_angle(servo_angle)
        time.sleep(0.5)  # Wait for servo to move and readings to stabilize
        distance = ultrasonic.get_distance()
        if distance > 0:  # valid reading
            occupancy_grid.update(angle, distance)
