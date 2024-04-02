import curses, os, sys, time
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
    for angle in range(-90, 91, 18):  # Simplified scanning process
        status = us.get_status_at(angle)
        scan_results.append(status)
    
    if all(status == 2 for status in scan_results):
        robot.forward(70)
    elif any(status == 0 for status in scan_results):
        robot.backward(50)
        time.sleep(1)
        if scan_results.index(0) < len(scan_results) / 2:
            robot.turn_right(70)
        else:
            robot.turn_left(70)
    else:
        robot.forward(50)
    robot.stop()

def main(window):
    curses.noecho()
    curses.cbreak()
    window.keypad(True)

    robot = Robot(config)
    us = Ultrasonic(Pin('D8'), Pin('D9'))  # Initialize Ultrasonic sensor, adjust pins as necessary
    selected_motor = None
    auto_mode = False  # Start in manual mode

    try:
        while True:
            if auto_mode:
                navigate_obstacles(robot, us)
                time.sleep(1)  # Adjust timing as necessary
            else:
                char = window.getch()

                if char == ord('q'):
                    break
                elif char == ord('m'):  # Toggle manual/auto mode
                    auto_mode = not auto_mode
                elif char == curses.KEY_UP or char == ord('w'):
                    robot.forward(100)
                elif char == curses.KEY_DOWN or char == ord('s'):
                    robot.backward(100)
                elif char == curses.KEY_LEFT or char == ord('a'):
                    robot.turn_left(100)
                elif char == curses.KEY_RIGHT or char == ord('d'):
                    robot.turn_right(100)
                elif char == ord(' '):
                    robot.stop()
                elif char == ord('1'):
                    selected_motor = "left_front"
                elif char == ord('2'):
                    selected_motor = "right_front"
                elif char == ord('3'):
                    selected_motor = "left_rear"
                elif char == ord('4'):
                    selected_motor = "right_rear"
                elif char == ord('+') and selected_motor:
                    robot.set_motor_power(selected_motor, 10)  # Placeholder, adjust your power control logic
                elif char == ord('-') and selected_motor:
                    robot.set_motor_power(selected_motor, -10)  # Placeholder, adjust your power control logic

    finally:
        curses.nocbreak()
        window.keypad(False)
        curses.echo()
        curses.endwin()
        robot.stop()

if __name__ == '__main__':
    curses.wrapper(main)
