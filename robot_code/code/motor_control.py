import os, sys
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

    def apply_motor_power(self, power, reverse=False):
        for key, motor in self.motors.items():
            direction_power = -power if (motor.is_reversed ^ reverse) else power
            motor.set_power(direction_power)

    def forward(self, power=100):
        self.apply_motor_power(power)

    def backward(self, power=100):
        self.apply_motor_power(power, reverse=True)

    def turn_left(self, power=100, pivot=False):
        if pivot:  # Pivot turn
            self.motors["left_front"].set_power(-power)
            self.motors["left_rear"].set_power(-power)
        else:  # Smooth turn
            self.motors["left_front"].set_power(power // 2)
            self.motors["left_rear"].set_power(power // 2)
        self.motors["right_front"].set_power(power)
        self.motors["right_rear"].set_power(power)

    def turn_right(self, power=100, pivot=False):
        self.motors["left_front"].set_power(power)
        self.motors["left_rear"].set_power(power)
        if pivot:  # Pivot turn
            self.motors["right_front"].set_power(-power)
            self.motors["right_rear"].set_power(-power)
        else:  # Smooth turn
            self.motors["right_front"].set_power(power // 2)
            self.motors["right_rear"].set_power(power // 2)

    def stop(self):
        for motor in self.motors.values():
            motor.set_power(0)
