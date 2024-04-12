import os, sys
# Adjust the sys.path to include the parent directory of robot_code
script_dir = os.path.dirname(__file__)
parent_dir = os.path.join(script_dir, '..', '..')
sys.path.append(os.path.abspath(parent_dir))
from config import config
from robot_code.modules.motor import Motor
from robot_code.modules.speed import Speed
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

        

    def set_motor_power(self, motor, power):
        # Correctly call set_power for each motor
        if motor == "left_front":
            self.motors["left_front"].set_power(power)
        elif motor == "right_front":
            self.motors["right_front"].set_power(power)
        elif motor == "left_rear":
            self.motors["left_rear"].set_power(power)
        elif motor == "right_rear":
            self.motors["right_rear"].set_power(power)

    # The rest of your methods remain unchanged

    '''      
    def set_all_motor_power(self, left_power, right_power):
        """Set power for all motors for coordinated movement."""
        self.motors["left_front"].set_power(left_power)
        self.motors["right_front"].set_power(right_power)
        self.motors["left_rear"].set_power(left_power)
        self.motors["right_rear"].set_power(right_power)
    
    '''
    '''
    def forward(self, power):
        self.motors["left_front"].set_power(-power+10)
        self.motors["left_rear"].set_power(power)
        self.motors["right_front"].set_power(-power+10)
        self.motors["right_rear"].set_power(power)
  
        
    def backward(self, power):
        self.motors["left_front"].set_power(power-10)
        self.motors["left_rear"].set_power(-power)
        self.motors["right_front"].set_power(power-10)
        self.motors["right_rear"].set_power(-power)
    '''
    def forward(self, power, neg_power):
            self.motors["left_front"].set_power(neg_power+10)
            self.motors["left_rear"].set_power(power)
            self.motors["right_front"].set_power(neg_power+10)
            self.motors["right_rear"].set_power(power)
  
        
    def backward(self, power, neg_power):
        self.motors["left_front"].set_power(power-10)
        self.motors["left_rear"].set_power(neg_power)
        self.motors["right_front"].set_power(power-10)
        self.motors["right_rear"].set_power(neg_power)

    def turn_left(self, power, turn_power_reduction=0.5):
            # Inside motors are the left motors when turning left
            inside_power = power * turn_power_reduction
            self.motors["left_front"].set_power(-inside_power)
            self.motors["left_rear"].set_power(-inside_power)
            self.motors["right_front"].set_power(power)
            self.motors["right_rear"].set_power(power)

    def turn_right(self, power, turn_power_reduction=0.5):
        # Inside motors are the right motors when turning right
        inside_power = power * turn_power_reduction
        self.motors["left_front"].set_power(power)
        self.motors["left_rear"].set_power(power)
        self.motors["right_front"].set_power(-inside_power)
        self.motors["right_rear"].set_power(-inside_power)


    def stop(self):
        self.motors["left_front"].set_power(0)
        self.motors["left_rear"].set_power(0)
        self.motors["right_front"].set_power(0)
        self.motors["right_rear"].set_power(0)

