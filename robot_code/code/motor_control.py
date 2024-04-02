# motor_control.py
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
        self.current_power = {key: 0 for key in self.motors}

    def set_motor_power(self, label, power):
        if label in self.motors:
            self.motors[label].set_power(power)
            self.current_power[label] = power

    def adjust_motor_speed(self, motor_label, delta):
        if motor_label in self.motors:
            new_power = max(min(self.current_power[motor_label] + delta, 100), -100)
            self.set_motor_power(motor_label, new_power)

    def get_motor_power(self, motor_label):
        if motor_label in self.current_power:
            return self.current_power[motor_label]
        return 0

    def forward(self, power=100):
        # Adjusted for consistent forward movement
        for key, motor in self.motors.items():
            motor.set_power(power if not motor.is_reversed else -power)
        self.current_power = {key: power for key in self.motors}

    def backward(self, power=100):
        # Adjusted for consistent backward movement
        for key, motor in self.motors.items():
            motor.set_power(-power if not motor.is_reversed else power)
        self.current_power = {key: -power for key in self.motors}

    def turn_left(self, power=100):
        # Assuming left motors need to reverse to turn left
        self.motors["left_front"].set_power(-power)
        self.motors["left_rear"].set_power(-power)
        self.motors["right_front"].set_power(power)
        self.motors["right_rear"].set_power(power)
        self.current_power = {
            "left_front": -power,
            "left_rear": -power,
            "right_front": power,
            "right_rear": power
        }

    def turn_right(self, power=100):
        # Assuming right motors need to reverse to turn right
        self.motors["left_front"].set_power(power)
        self.motors["left_rear"].set_power(power)
        self.motors["right_front"].set_power(-power)
        self.motors["right_rear"].set_power(-power)
        self.current_power = {
            "left_front": power,
            "left_rear": power,
            "right_front": -power,
            "right_rear": -power
        }

    def stop(self):
        for motor in self.motors.values():
            motor.set_power(0)
        self.current_power = {key: 0 for key in self.motors}
