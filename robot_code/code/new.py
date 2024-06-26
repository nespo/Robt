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

'''
def cmd_vel_callback(data, robot):
    # Velocity conversion: Adjust this logic based on your robot's specifications
    linear_speed = data.linear.x  # m/s
    angular_speed = data.angular.z  # rad/s

    # Differential drive robot formula for wheel speeds:
    left_power = linear_speed - angular_speed
    right_power = linear_speed + angular_speed

    # Adjust power values based on your robot's specific needs
    left_power_scaled = max(min(left_power * 100, 100), -100)
    right_power_scaled = max(min(right_power * 100, 100), -100)

    # Apply correction for motor direction
    robot.set_motor_power("left_front", -left_power_scaled)
    robot.set_motor_power("left_rear", -left_power_scaled)
    robot.set_motor_power("right_front", right_power_scaled)
    robot.set_motor_power("right_rear", right_power_scaled)
'''
def cmd_vel_callback(data, robot):
    # Velocity conversion: Extract linear and angular components
    linear_speed = data.linear.x  # m/s
    angular_speed = data.angular.z  # rad/s

    # Calculate powers for the rear wheels using differential drive formula
    rear_left_power = linear_speed - angular_speed
    rear_right_power = linear_speed + angular_speed

    # Scale the powers to the motor input range and clamp them to max/min values
    rear_left_power_scaled = max(min(rear_left_power * 100, 100), -100)
    rear_right_power_scaled = max(min(rear_right_power * 100, 100), -100)

    # Front wheels get 10-15 units less power than the rear wheels
    # Using an average reduction of 12.5 for simplicity
    power_reduction = 12.5
    front_left_power_scaled = max(min(rear_left_power_scaled - power_reduction, 100), -100)
    front_right_power_scaled = max(min(rear_right_power_scaled - power_reduction, 100), -100)

    # Apply direction corrections for your robot's specific configuration:
    # Front wheels rotate counterclockwise for forward movement (negative power)
    # Rear wheels rotate clockwise for forward movement (positive power)
    #robot.set_motor_powers(-front_left_power_scaled, rear_left_power_scaled,-front_right_power_scaled, rear_right_power_scaled)
    robot.set_motor_power("left_front", -rear_left_power_scaled+10)
    robot.set_motor_power("left_rear", -rear_left_power_scaled)
    robot.set_motor_power("right_front", rear_right_power_scaled+10)
    robot.set_motor_power("right_rear", rear_right_power_scaled)

# Ensure that your robot class has a method `set_motor_powers(front_left, rear_left, front_right, rear_right)`
# which is responsible for sending these power values to the respective motor drivers.




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
