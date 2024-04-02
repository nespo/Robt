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
        self.current_speed = 0

    def _adjust_power_for_smooth_control(self, target_power, step=10):
        if target_power > self.current_speed:
            for power in range(self.current_speed, target_power, step):
                self.apply_motor_power(power)
                time.sleep(0.1)
        elif target_power < self.current_speed:
            for power in range(self.current_speed, target_power, -step):
                self.apply_motor_power(power)
                time.sleep(0.1)
        self.current_speed = target_power

    def apply_motor_power(self, power, reverse=False):
        for key, motor in self.motors.items():
            direction_power = -power if (motor.is_reversed ^ reverse) else power
            motor.set_power(direction_power)

    def forward(self, power=100, smooth_start=True):
        if smooth_start:
            self._adjust_power_for_smooth_control(power)
        else:
            self.apply_motor_power(power)

    def backward(self, power=100, smooth_start=True):
        if smooth_start:
            self._adjust_power_for_smooth_control(-power)
        else:
            self.apply_motor_power(power, reverse=True)

    def turn_left(self, power=100, pivot=False, dynamic_steer=False):
        adjust_power = power // 2 if dynamic_steer else power
        if pivot:  # Pivot turn
            self.motors["left_front"].set_power(-adjust_power)
            self.motors["left_rear"].set_power(-adjust_power)
        else:  # Smooth turn
            self.motors["left_front"].set_power(adjust_power // 2)
            self.motors["left_rear"].set_power(adjust_power // 2)
        self.motors["right_front"].set_power(adjust_power)
        self.motors["right_rear"].set_power(adjust_power)

    def turn_right(self, power=100, pivot=False, dynamic_steer=False):
        adjust_power = power // 2 if dynamic_steer else power
        if pivot:  # Pivot turn
            self.motors["right_front"].set_power(-adjust_power)
            self.motors["right_rear"].set_power(-adjust_power)
        else:  # Smooth turn
            self.motors["right_front"].set_power(adjust_power // 2)
            self.motors["right_rear"].set_power(adjust_power // 2)
        self.motors["left_front"].set_power(adjust_power)
        self.motors["left_rear"].set_power(adjust_power)

    def stop(self):
        self._adjust_power_for_smooth_control(0)

    # Implement any additional functions like path correction here based on sensor feedback.
