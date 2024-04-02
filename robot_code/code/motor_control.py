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
                                 reverse=config["motors"]["right_front"]["reverse"]),
            "left_rear": Motor(pin_pwm=config["motors"]["left_rear"]["pin_pwm"], 
                               pin_dir=config["motors"]["left_rear"]["pin_dir"], 
                               reverse=config["motors"]["left_rear"]["reverse"]),
            "right_rear": Motor(pin_pwm=config["motors"]["right_rear"]["pin_pwm"], 
                                pin_dir=config["motors"]["right_rear"]["pin_dir"], 
                                reverse=config["motors"]["right_rear"]["reverse"]),
        }

    def apply_motor_power(self, power, reverse=False, gradual=False):
        target_power = -power if reverse else power
        if gradual:
            for step in range(0, target_power, 10 if target_power > 0 else -10):
                for key, motor in self.motors.items():
                    motor.set_power(step if not motor.is_reversed else -step)
                time.sleep(0.05)  # Adjust the sleep time for smoother acceleration
        else:
            for key, motor in self.motors.items():
                motor.set_power(target_power if not motor.is_reversed else -target_power)

    def forward(self, power=100, gradual=False):
        self.apply_motor_power(power, gradual=gradual)

    def backward(self, power=100, gradual=False):
        self.apply_motor_power(power, reverse=True, gradual=gradual)

    def turn(self, direction, power=100, radius=0):
        if direction == "left":
            left_power = max(0, power - radius)
            right_power = power
        else:  # direction == "right"
            left_power = power
            right_power = max(0, power - radius)
        
        self.motors["left_front"].set_power(left_power)
        self.motors["left_rear"].set_power(left_power)
        self.motors["right_front"].set_power(right_power)
        self.motors["right_rear"].set_power(right_power)

    def stop(self, gradual=False):
        if gradual:
            self.apply_motor_power(0, gradual=True)
        else:
            for motor in self.motors.values():
                motor.set_power(0)
