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
    wheel_rad = 0.03  # Radius of the wheel in meters
    distance_of_wheels = 0.14  # Distance between wheels in meters

    # Extract linear and angular velocities from the received data
    linear_speed = data.linear.x  # Forward/backward speed (m/s)
    angular_speed = data.angular.z  # Turning speed (rad/s)

    # Compute wheel speeds in rad/s
    wl = (linear_speed - (angular_speed * distance_of_wheels) / 2) / wheel_rad
    wr = (linear_speed + (angular_speed * distance_of_wheels) / 2) / wheel_rad

    # Scale the wheel speeds to PWM values
    max_wheel_speed = 5  # Maximum reasonable wheel speed in rad/s
    scale_factor = 100 / max_wheel_speed  # Scale to max power of 100 for forwards/backwards

    # Apply scaling
    pwm_left = int(wl * scale_factor)
    pwm_right = int(wr * scale_factor)

    # Ensure PWM values are within the allowed range
    pwm_left = max(-100, min(100, pwm_left))
    pwm_right = max(-100, min(100, pwm_right))

    # Assigning motor powers based on the velocities and handling directions
    if linear_speed > 0:  # Forward
        robot.forward(max(abs(pwm_left), abs(pwm_right)))
    elif linear_speed < 0:  # Backward
        robot.backward(max(abs(pwm_left), abs(pwm_right)))
    else:
        robot.stop()  # No movement




    # Print the computed powers for debugging
    print("LFP:", pwm_left, "RFP:", pwm_right)





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
