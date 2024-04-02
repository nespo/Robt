import os, sys
# Adjust the sys.path to include the parent directory of robot_code
script_dir = os.path.dirname(__file__)
parent_dir = os.path.join(script_dir, '..', '..')
sys.path.append(os.path.abspath(parent_dir))
from config import config
from robot_code.modules.motor import Motor
import RPi.GPIO as GPIO

class Robot:
    def __init__(self, config):
        GPIO.setmode(GPIO.BCM)
        self.motors = {
            "left_front": Motor(pin_pwm=config["motors"]["left_front"]["pin_pwm"],
                                pin_dir=config["motors"]["left_front"]["pin_dir"],
                                reverse=config["motors"]["left_front"]["reverse"]),
            "right_front": Motor(pin_pwm=config["motors"]["right_front"]["pin_pwm"],
                                 pin_dir=config["motors"]["right_front"]["pin_dir"],
                                 reverse=config["motors"]["right_front"]["reverse"]),
            "left_rear": Motor(pin_pwm=config["motors"]["left_rear"]["pin_pwm"],
                               pin_dir=config["motors"]["left_rear"]["pin_dir"],
                               reverse=config["motors"]["left_rear"]["reverse"]),
            "right_rear": Motor(pin_pwm=config["motors"]["right_rear"]["pin_pwm"],
                                pin_dir=config["motors"]["right_rear"]["pin_dir"],
                                reverse=config["motors"]["right_rear"]["reverse"]),
        }

    def set_all_motor_power(self, left_power, right_power):
        """Set power for all motors for coordinated movement."""
        self.motors["left_front"].set_power(left_power)
        self.motors["right_front"].set_power(right_power)
        self.motors["left_rear"].set_power(left_power)
        self.motors["right_rear"].set_power(right_power)

    def forward(self, speed=100):
        """Move the car forward."""
        self.set_all_motor_power(speed, speed)

    def backward(self, speed=100):
        """Move the car backward."""
        self.set_all_motor_power(-speed, -speed)

    def stop(self):
        """Stop the car."""
        self.set_all_motor_power(0, 0)

    def turn_left(self, speed=100, sharpness=0.5):
        """Turn the car left with adjustable sharpness."""
        left_speed = speed * (1 - sharpness)
        right_speed = speed
        self.set_all_motor_power(left_speed, right_speed)

    def turn_right(self, speed=100, sharpness=0.5):
        """Turn the car right with adjustable sharpness."""
        right_speed = speed * (1 - sharpness)
        left_speed = speed
        self.set_all_motor_power(left_speed, right_speed)

