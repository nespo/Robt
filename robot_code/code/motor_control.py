import os, sys
# Adjust the sys.path to include the parent directory of robot_code
script_dir = os.path.dirname(__file__)
parent_dir = os.path.join(script_dir, '..', '..')
sys.path.append(os.path.abspath(parent_dir))
from config import config
from robot_code.modules.motor import Motor
import RPi.GPIO as GPIO
import time

class Robot:
    def __init__(self, config):
        GPIO.setmode(GPIO.BCM)
        self.motors = {
            "left_front": Motor(pin_pwm=config["motors"]["left_front"]["pin_pwm"], 
                                pin_dir=config["motors"]["left_front"]["pin_dir"], 
                                reverse=config["motors"]["left_front"]["reverse"]),
            "right_front": Motor(pin_pwm=config["motors"]["right_front"]["pin_pwm"], 
                                 pin_dir=config["motors"]["right_front"]["pin_dir"], 
                                 reverse=not config["motors"]["right_front"]["reverse"]), # Invert logic based on observed behavior
            "left_rear": Motor(pin_pwm=config["motors"]["left_rear"]["pin_pwm"], 
                               pin_dir=config["motors"]["left_rear"]["pin_dir"], 
                               reverse=config["motors"]["left_rear"]["reverse"]),
            "right_rear": Motor(pin_pwm=config["motors"]["right_rear"]["pin_pwm"], 
                                pin_dir=config["motors"]["right_rear"]["pin_dir"], 
                                reverse=not config["motors"]["right_rear"]["reverse"]), # Invert logic based on observed behavior
        }

    def apply_motor_power(self, power, reverse=False):
        for key, motor in self.motors.items():
            # Adjusting logic to directly apply correct power based on the reverse flag
            motor.set_power(-power if reverse else power)

    def forward(self, power=100):
        self.apply_motor_power(power, reverse=False)

    def backward(self, power=100):
        self.apply_motor_power(power, reverse=True)

    def turn_left(self, power=100, pivot=False):
        # Adjusting turning logic to ensure proper coordination between wheels
        left_power = -power if pivot else power / 2
        right_power = power
        self.motors["left_front"].set_power(left_power)
        self.motors["left_rear"].set_power(left_power)
        self.motors["right_front"].set_power(right_power)
        self.motors["right_rear"].set_power(right_power)

    def turn_right(self, power=100, pivot=False):
        # Adjusting turning logic to ensure proper coordination between wheels
        right_power = -power if pivot else power / 2
        left_power = power
        self.motors["right_front"].set_power(right_power)
        self.motors["right_rear"].set_power(right_power)
        self.motors["left_front"].set_power(left_power)
        self.motors["left_rear"].set_power(left_power)

    def stop(self):
        for motor in self.motors.values():
            motor.set_power(0)

