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
    
    # Adjust speeds to power settings, considering the robot's configuration
    if linear_speed > 0:  # Moving forward
        left_front_power = -(linear_speed * 100) + 10
        right_front_power = -(linear_speed * 100) + 10
        left_rear_power = (linear_speed * 100)
        right_rear_power = (linear_speed * 100)
    elif linear_speed < 0:  # Moving backward
        left_front_power = (abs(linear_speed) * 100) - 10
        right_front_power = (abs(linear_speed) * 100) - 10
        left_rear_power = -(abs(linear_speed) * 100)
        right_rear_power = -(abs(linear_speed) * 100)
    else:  # Staying still or turning in place
        left_front_power = 0
        right_front_power = 0
        left_rear_power = 0
        right_rear_power = 0

    # Adjust for turning by modifying the power based on angular speed
    if angular_speed > 0:  # Turning left
        left_front_power -= angular_speed * 50  # Reduce power for turning
        left_rear_power -= angular_speed * 50
    elif angular_speed < 0:  # Turning right
        right_front_power += angular_speed * 50  # Reduce power for turning (note: angular_speed is negative)
        right_rear_power += angular_speed * 50

    # Apply power settings to the motors
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
