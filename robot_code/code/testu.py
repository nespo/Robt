import unittest
from unittest.mock import MagicMock, patch
import sys

# Mock external modules not supported on the testing platform
sys.modules['RPi.GPIO'] = MagicMock()
sys.modules['smbus'] = MagicMock()
sys.modules['smbus2'] = MagicMock()

# Import your modules after applying mocks
from global_2 import NavigationSystem, dynamic_navigation, calculate_bearing, plot_environment

class TestNavigationSystem(unittest.TestCase):
    @patch('serial.Serial')  # Patch serial.Serial to prevent real serial communication
    def setUp(self, mock_serial):
        # Configure the MagicMock to simulate a serial port
        mock_serial.return_value.read.return_value = b'Your expected serial data here'
        mock_serial.return_value.write = MagicMock()

        # Initialize the navigation system with a smaller grid for testing
        self.nav_system = NavigationSystem(grid_resolution=0.5, grid_size=100)
        self.robot = MagicMock()  # Simulate the Robot object

    def test_gps_to_utm_conversion(self):
        lat, lon = 40.7128, -74.0060  # Coordinates for New York City
        expected_utm = (583960.2485515354, 4507069.984907427)
        utm_coordinates = self.nav_system.gps_to_utm(lat, lon)
        self.assertAlmostEqual(utm_coordinates[0], expected_utm[0], places=1)
        self.assertAlmostEqual(utm_coordinates[1], expected_utm[1], places=1)

    def test_utm_to_grid_conversion(self):
        self.nav_system.grid_origin = (500000, 4600000)
        utm_x, utm_y = 500100, 4600100
        expected_grid = (200, 200)
        grid_coordinates = self.nav_system.utm_to_grid(utm_x, utm_y)
        self.assertEqual(grid_coordinates, expected_grid)

    def test_a_star_search(self):
        start = (10, 10)
        goal = (10, 20)
        self.nav_system.grid[15, 10:20] = 1  # Obstacle
        path = self.nav_system.a_star_search(start, goal, self.nav_system.grid)
        self.assertIsInstance(path, list)

    def test_bearing_calculation(self):
        pointA = (40.7128, -74.0060)
        pointB = (40.7138, -74.0070)
        bearing = calculate_bearing(pointA, pointB)
        expected_bearing = 303.0
        self.assertAlmostEqual(bearing, expected_bearing, places=0)

    @patch('global_2.get_current_gps')
    @patch('global_2.get_current_heading')
    def test_dynamic_navigation(self, mock_get_current_heading, mock_get_current_gps):
        mock_get_current_gps.return_value = (40.7128, -74.0060)
        mock_get_current_heading.return_value = 0
        start_lat, start_lon = 40.7128, -74.0060
        goal_lat, goal_lon = 40.7138, -74.0070
        self.dynamic_navigation(self.nav_system, start_lat, start_lon, goal_lat, goal_lon, self.robot)
        self.assertTrue(mock_get_current_heading.called)
        self.assertTrue(self.robot.turn_left.called or self.robot.turn_right.called)

    @patch('matplotlib.pyplot.show')
    def test_plot_environment(self, mock_show):
        goal_utm = self.nav_system.gps_to_utm(40.7138, -74.0070)
        self.plot_environment(self.nav_system, goal_utm)
        mock_show.assert_called_once()

if __name__ == '__main__':
    unittest.main()
