import curses, os, sys
# Adjust the sys.path to include the parent directory of robot_code
script_dir = os.path.dirname(__file__)
parent_dir = os.path.join(script_dir, '..', '..')
sys.path.append(os.path.abspath(parent_dir))

from robot_code.code.motor_control import Robot
from config import config

def main(window):
    curses.noecho()
    curses.cbreak()
    window.keypad(True)

    robot = Robot(config)
    selected_motor = None

    try:
        while True:
            char = window.getch()

            if char == ord('q'):
                break
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
                # Increase the selected motor's speed
                # Placeholder for the method to increase speed; implement according to your setup
                robot.adjust_motor_speed(selected_motor, 10)
            elif char == ord('-') and selected_motor:
                # Decrease the selected motor's speed
                # Placeholder for the method to decrease speed; implement according to your setup
                robot.adjust_motor_speed(selected_motor, -10)

    finally:
        curses.nocbreak()
        window.keypad(False)
        curses.echo()
        curses.endwin()
        robot.stop()

if __name__ == '__main__':
    curses.wrapper(main)
