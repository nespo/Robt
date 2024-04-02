import os, sys
# Adjust the sys.path to include the parent directory of robot_code (assuming it's one level up)
script_dir = os.path.dirname(__file__)
parent_dir = os.path.join(script_dir, '..')
sys.path.append(os.path.abspath(parent_dir))
from config import config
from robot_code.modules.motor import Motor
import RPi.GPIO as GPIO
import time

class Robot:
  """
  Class to manage motor control functionality
  """
  def __init__(self, config):
    GPIO.setmode(GPIO.BCM)
    self.motors = {
      key: Motor(pin_pwm=config["motors"][key]["pin_pwm"],
                  pin_dir=config["motors"][key]["pin_dir"],
                  reverse=config["motors"][key]["reverse"])
      for key in config["motors"]}

  def apply_motor_power(self, power, reverse=False):
    """
    Applies power to all motors with direction control
    """
    for motor in self.motors.values():
      direction_power = power if (motor.is_reversed ^ reverse) else -power
      motor.set_power(direction_power)

  def forward(self, power=100, smooth_start=True):
    """
    Moves the robot forward
    """
    if smooth_start:
      self._adjust_power_for_smooth_control(power)
    else:
      self.apply_motor_power(power)

  def backward(self, power=100, smooth_start=True):
    """
    Moves the robot backward
    """
    if smooth_start:
      self._adjust_power_for_smooth_control(-power)
    else:
      self.apply_motor_power(power, reverse=True)

  def turn_left(self, power=100, pivot=False, dynamic_steer=False):
    """
    Turns the robot left
    """
    adjust_power = power // 2 if dynamic_steer else power
    if pivot:
      self.motors["left_front"].set_power(-adjust_power)
      self.motors["left_rear"].set_power(-adjust_power)
    else:
      self.motors["left_front"].set_power(-adjust_power)  # Inverted for left side
      self.motors["left_rear"].set_power(-adjust_power)
      self.motors["right_front"].set_power(adjust_power)
      self.motors["right_rear"].set_power(adjust_power)

  def turn_right(self, power=100, pivot=False, dynamic_steer=False):
    """
    Turns the robot right
    """
    adjust_power = power // 2 if dynamic_steer else power
    if pivot:
      self.motors["right_front"].set_power(-adjust_power)
      self.motors["right_rear"].set_power(-adjust_power)
    else:
      self.motors["right_front"].set_power(-adjust_power)  # Inverted for right side
      self.motors["right_rear"].set_power(-adjust_power)
      self.motors["left_front"].set_power(adjust_power)
      self.motors["left_rear"].set_power(adjust_power)

  def stop(self):
    """
    Stops all motors smoothly
    """
    self._adjust_power_for_smooth_control(0)

  def _adjust_power_for_smooth_control(self, target_power, step=10):
    """
    Gradually adjusts motor power for smoother control
    """
    current_power = self.get_current_power()
    if target_power > current_power:
      for power in range(current_power, target_power, step):
        self.apply_motor_power(power)
        time.sleep(0.1)
    elif target_power < current_power:
      for power in range(current_power, target_power, -step):
        self.apply_motor_power(power)
        time.sleep(0.1)
    self.motors["left_front"].set_power(target_power)  # Update
