import curses
import os
import sys
import time

# Adjust the sys.path to include the parent directory of robot_code
script_dir = os.path.dirname(__file__)
parent_dir = os.path.join(script_dir, '..', '..')
sys.path.append(os.path.abspath(parent_dir))

from robot_code.code.motor_control import Robot
from robot_code.code.config import config
from robot_code.modules.ultrasonic import Ultrasonic
from robot_code.modules.pin import Pin

def navigate_obstacles(robot, us):
    scan_results = []
    for angle in range(-90, 91, 18):
        status = us.get_status_at(angle)
        scan_results.append(status)

    # Automatically move forward but adjust based on obstacles
    if all(status == 2 for status in scan_results):
        robot.forward(70)
    elif any(status == 0 for status in scan_results):
        robot.backward(50)
        time.sleep(1)
        if scan_results.index(0) < len(scan_results) / 2:
            robot.turn_right(70)
        else:
            robot.turn_left(70)
        time.sleep(1)
    else:
        robot.forward(50)
    # No need to stop the robot here if we want continuous movement in auto mode

def main(window):
    curses.noecho()
    curses.cbreak()
    window.nodelay(True)
    window.keypad(True)

    robot = Robot(config)
    us = Ultrasonic(Pin('D8'), Pin('D9'))
    auto_mode = False

    try:
        while True:
            char = window.getch()
            if char != -1:  # User input detected
                if char == ord('q'):
                    break
                elif char == ord('m'):  # Toggle manual/auto mode
                    auto_mode = not auto_mode
                    if auto_mode:
                        # When switching to auto mode, start moving forward by default
                        robot.forward(50)
                    else:
                        # Stop the robot when switching back to manual mode
                        robot.stop()
                    window.nodelay(True)  # Keep non-blocking mode for continuous checks
                    continue

            if auto_mode:
                navigate_obstacles(robot, us)
                time.sleep(1)  # Adjust timing as necessary
            else:  # Manual control
                if char == curses.KEY_UP or char == ord('w'):
                    robot.forward(100)
                elif char == curses.KEY_DOWN or char == ord('s'):
                    robot.backward(100)
                elif char == curses.KEY_LEFT or char == ord('a'):
                    robot.turn_left(90)
                elif char == curses.KEY_RIGHT or char == ord('d'):
                    robot.turn_right(90)
                elif char == ord(' '):
                    robot.stop()

    finally:
        # Clean up
        curses.nocbreak()
        window.nodelay(False)
        window.keypad(False)
        curses.echo()
        curses.endwin()
        robot.stop()

if __name__ == '__main__':
    curses.wrapper(main)
