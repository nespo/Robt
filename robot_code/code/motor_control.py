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
            "left": Motor(pin_pwm=config["motors"]["left_front"]["pin_pwm"],
                          pin_dir=config["motors"]["left_front"]["pin_dir"],
                          reverse=config["motors"]["left_front"]["reverse"]),
            "right": Motor(pin_pwm=config["motors"]["right_front"]["pin_pwm"],
                           pin_dir=config["motors"]["right_front"]["pin_dir"],
                           reverse=config["motors"]["right_front"]["reverse"])
        }
        # Simplify by using just front motors for direction and speed control

    def set_motor_power(self, left_power, right_power):
        self.motors["left"].set_power(left_power)
        self.motors["right"].set_power(right_power)

    def forward(self, speed=100):
        self.set_motor_power(speed, speed)

    def backward(self, speed=100):
        self.set_motor_power(-speed, -speed)

    def stop(self):
        self.set_motor_power(0, 0)

    def turn_left(self, speed=100, pivot=False):
        if pivot:
            self.set_motor_power(-speed, speed)
        else:
            self.set_motor_power(speed * 0.5, speed)

    def turn_right(self, speed=100, pivot=False):
        if pivot:
            self.set_motor_power(speed, -speed)
        else:
            self.set_motor_power(speed, speed * 0.5)

