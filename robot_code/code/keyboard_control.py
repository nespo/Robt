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

    try:
        while True:
            char = window.getch()

            if char == ord('q'):
                break
            elif char == curses.KEY_UP or char == ord('w'):
                robot.forward(100, gradual=True)  # Use gradual acceleration for forward
            elif char == curses.KEY_DOWN or char == ord('s'):
                robot.backward(100, gradual=True)  # Use gradual acceleration for backward
            elif char == curses.KEY_LEFT or char == ord('a'):
                # For smooth left turn, adjust the radius as needed
                robot.turn("left", 100, radius=50)
            elif char == curses.KEY_RIGHT or char == ord('d'):
                # For smooth right turn, adjust the radius as needed
                robot.turn("right", 100, radius=50)
            elif char == ord(' '):
                robot.stop(gradual=True)  # Gradual stop
            # Adjusting individual motor speeds removed for simplicity. Reimplement if needed.

    finally:
        curses.nocbreak()
        window.keypad(False)
        curses.echo()
        curses.endwin()
        robot.stop(gradual=True)  # Ensure a gradual stop on exit

if __name__ == '__main__':
    curses.wrapper(main)
