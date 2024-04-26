import threading
import math
import time

import os
import sys

# Correct library import paths
script_dir = os.path.dirname(os.path.realpath(__file__))
parent_dir = os.path.join(script_dir, '..', '..')
sys.path.append(os.path.abspath(parent_dir))
from robot_code.modules.nav import get_current_gps, get_current_heading
from robot_code.modules.motor import Motor
from robot_code.modules.speed import Speed
import RPi.GPIO as GPIO

# Constants similar to C++ #define statements
GPS_UPDATE_INTERVAL = 5000  # Time interval for GPS update in milliseconds
MOTOR_A_OFFSET = 10
MOTOR_B_OFFSET = 10
DEG_TO_RAD = 0.017453292519943295
RAD_TO_DEG = 57.29577951308232
RC_NEUTRAL = 1500
RC_MAX = 2000
RC_MIN = 1000

class Robot:
    def __init__(self, config):
        GPIO.setmode(GPIO.BCM)
        self.motors = {
            "left_front": Motor(config["motors"]["left_front"]["pin_pwm"],
                                config["motors"]["left_front"]["pin_dir"],
                                config["motors"]["left_front"]["reverse"]),
            "right_front": Motor(config["motors"]["right_front"]["pin_pwm"],
                                 config["motors"]["right_front"]["pin_dir"],
                                 config["motors"]["right_front"]["reverse"]),
            "left_rear": Motor(config["motors"]["left_rear"]["pin_pwm"],
                               config["motors"]["left_rear"]["pin_dir"],
                               config["motors"]["left_rear"]["reverse"]),
            "right_rear": Motor(config["motors"]["right_rear"]["pin_pwm"],
                                config["motors"]["right_rear"]["pin_dir"],
                                config["motors"]["right_rear"]["reverse"]),
        }

    def drive_to(self, destination, timeout):
        start_time = time.time()
        while True:
            current_loc = get_current_gps()
            current_heading = get_current_heading()
            distance = self.geo_distance(current_loc, destination)
            bearing = self.geo_bearing(current_loc, destination)
            required_heading = (bearing - current_heading) % 360

            self.drive(distance, required_heading)

            if distance <= 1 or (time.time() - start_time) > timeout:
                self.stop()
                break

    def geo_distance(self, loc1, loc2):
        lat1, lon1 = loc1
        lat2, lon2 = loc2
        R = 6371000
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        a = math.sin(delta_phi / 2.0)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c

    def geo_bearing(self, loc1, loc2):
        lat1, lon1 = loc1
        lat2, lon2 = loc2
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        delta_lambda = math.radians(lon2 - lon1)
        x = math.sin(delta_lambda) * math.cos(phi2)
        y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)
        return math.atan2(x, y) * RAD_TO_DEG

    def drive(self, distance, turn):
        full_speed = 100
        stop_speed = 0
        if distance < 8:
            speed = stop_speed + ((full_speed - stop_speed) * (distance / 8))
        else:
            speed = full_speed

        # Apply the turn calculations based on the required heading adjustment
        t_modifier = (180 - abs(turn)) / 180
        auto_steering_adjustment = (t_modifier, 1) if turn < 0 else (1, t_modifier)

        speed_a = self.map_speed(speed, RC_NEUTRAL, RC_MIN, *auto_steering_adjustment[0])
        speed_b = self.map_speed(speed, RC_NEUTRAL, RC_MAX, *auto_steering_adjustment[1])
        self.set_speed('left_front', speed_a)
        self.set_speed('right_front', speed_b)

    def map_speed(self, speed, neutral, limit, adjustment_factor=1):
        return int((limit - neutral) * adjustment_factor + neutral)

    def stop(self):
        for motor in self.motors.values():
            motor.set_power(0)

# Main program setup
if __name__ == "__main__":
    config = {"motors": {
        "left_front": {"pin_pwm": "P13", "pin_dir": "D4", "reverse": False},
        "right_front": {"pin_pwm": "P12", "pin_dir": "D5", "reverse": False},
        "left_rear": {"pin_pwm": "P8", "pin_dir": "D11", "reverse": False},
        "right_rear": {"pin_pwm": "P9", "pin_dir": "D15", "reverse": False}
    },}
    robot = Robot(config)
    destination = (62.878815, 27.637536)  # Example coordinates
    robot.drive_to(destination, 300)  # Drive to destination with a timeout of 300 seconds
