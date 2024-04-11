#!/usr/bin/env python3
import curses
import os
import sys
import time

# ROS Python API
sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')

import rospy
from geometry_msgs.msg import Twist

# Include the robot_code directory
script_dir = os.path.dirname(__file__)
parent_dir = os.path.join(script_dir, '..', '..')
sys.path.append(os.path.abspath(parent_dir))

from robot_code.code.motor_control import Robot
from robot_code.code.config import config
from robot_code.modules.ultrasonic import Ultrasonic
from robot_code.modules.pin import Pin

def cmd_vel_callback(data, robot):
    # Extract linear and angular velocities
    linear_speed = data.linear.x  # m/s
    angular_speed = data.angular.z  # rad/s

    # Default motor power adjustments
    adjust_power = 10
    turn_power_reduction = 0.5

    # Determine the base power from linear speed
    base_power = linear_speed * 100  # Scale as necessary

    # Calculate differential for turning based on angular speed
    turn_difference = angular_speed * 50  # Adjust the factor as necessary for your robot

    # Calculate individual motor powers
    left_front_power = -base_power + adjust_power - (turn_difference * turn_power_reduction)
    left_rear_power = base_power - (turn_difference * turn_power_reduction)
    right_front_power = -base_power + adjust_power + (turn_difference * turn_power_reduction)
    right_rear_power = base_power + (turn_difference * turn_power_reduction)

    # Apply power adjustments to the motors
    robot.set_motor_power("left_front", max(min(left_front_power, 100), -100))
    robot.set_motor_power("left_rear", max(min(left_rear_power, 100), -100))
    robot.set_motor_power("right_front", max(min(right_front_power, 100), -100))
    robot.set_motor_power("right_rear", max(min(right_rear_power, 100), -100))



def main(window):
    rospy.init_node('robot_controller', anonymous=True)

    robot = Robot(config)
    us = Ultrasonic(Pin('D8'), Pin('D9'))

    rospy.Subscriber("/cmd_vel", Twist, lambda data: cmd_vel_callback(data, robot))

    curses.noecho()
    curses.cbreak()
    window.nodelay(True)
    window.keypad(True)

    try:
        while not rospy.is_shutdown():
            char = window.getch()
            if char == ord('q'):  # Quit command
                break
            # Additional manual controls or functionalities can be added here

            # Your existing navigation logic can go here; for now, we rely on /cmd_vel

            time.sleep(0.1)  # Loop rate control
    finally:
        robot.stop()  # Ensure the robot stops on script exit or interrupt
        curses.nocbreak()
        window.nodelay(False)
        window.keypad(False)
        curses.echo()
        curses.endwin()

if __name__ == '__main__':
    curses.wrapper(main)
