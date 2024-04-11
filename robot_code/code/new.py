#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import curses
import os
import sys
import time

# Adding ROS Python API
sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')

# Adjust sys.path to include the robot_code directory
script_dir = os.path.dirname(__file__)
parent_dir = os.path.join(script_dir, '..', '..')
sys.path.append(os.path.abspath(parent_dir))

from robot_code.code.motor_control import Robot
from robot_code.code.config import config
from robot_code.modules.ultrasonic import Ultrasonic
from robot_code.modules.pin import Pin

def cmd_vel_callback(data, robot):
    # Calculate differential drive parameters from Twist message
    linear_speed = data.linear.x  # m/s
    angular_speed = data.angular.z  # rad/s

    # Convert to power levels for PiCar 4WD
    left_power = max(min((linear_speed - angular_speed) * 100, 100), -100)
    right_power = max(min((linear_speed + angular_speed) * 100, 100), -100)

    robot.set_motor_power("left_front", left_power)
    robot.set_motor_power("left_rear", left_power)
    robot.set_motor_power("right_front", right_power)
    robot.set_motor_power("right_rear", right_power)

def navigate_obstacles(robot, us):
    scan_results = []
    for angle in range(-90, 91, 18):
        status = us.get_status_at(angle)
        scan_results.append(status)

    # Obstacle avoidance logic
    if any(status == 0 for status in scan_results):
        robot.backward(50)
        time.sleep(1)
        direction = scan_results.index(0) < len(scan_results) / 2
        robot.turn_right(70) if direction else robot.turn_left(70)
        time.sleep(1)
        robot.forward(50)

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

            # Optional: Add manual control or other functionalities here

            navigate_obstacles(robot, us)
            time.sleep(0.1)  # Loop rate control
    finally:
        curses.nocbreak()
        window.nodelay(False)
        window.keypad(False)
        curses.echo()
        curses.endwin()
        robot.stop()  # Ensure robot stops on script termination or interrupt

if __name__ == '__main__':
    curses.wrapper(main)
