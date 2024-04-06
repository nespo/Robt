import curses
import os
import sys
import time
import threading
import cv2
import torch
import json
from pathlib import Path
from datetime import datetime
import numpy as np

# Adjust the sys.path to include the parent directory of robot_code
script_dir = os.path.dirname(__file__)
parent_dir = os.path.join(script_dir, '..', '..')
sys.path.append(os.path.abspath(parent_dir))

from robot_code.code.motor_control import Robot
from robot_code.code.config import config
from robot_code.modules.ultrasonic import Ultrasonic
from robot_code.modules.pin import Pin
# Import Servo and PWM if necessary for Ultrasonic operation
from robot_code.modules.servo import Servo
from robot_code.modules.pwm import PWM

# The following functions are from your camera and ultrasonic sensor integration code
def setup_directories(base_path: Path) -> tuple:
    """
    Ensure the directories for storing images and detection data exist.
    """
    image_dir = base_path / 'images'
    detection_dir = base_path / 'detections'
    image_dir.mkdir(parents=True, exist_ok=True)
    detection_dir.mkdir(parents=True, exist_ok=True)
    return image_dir, detection_dir

def save_detection_data(detection_dir: Path, frame_id: int, detected_objects: list, distance: float):
    """
    Modified to serialize detected_objects properly for JSON.
    """
    detection_info = {
        'frame_id': frame_id,
        'timestamp': datetime.now().strftime("%Y%m%d_%H%M%S%f"),
        'detections': detected_objects,
        'distance': distance
    }
    detections_json_path = detection_dir / f'detections_{frame_id:04d}.json'
    with detections_json_path.open('w') as f:
        json.dump(detection_info, f, default=str)  # Use default=str to handle non-serializable data

def process_stream(stream_url: str, model, ultrasonic_sensor):
    base_path = Path(__file__).resolve().parent.parent
    image_dir, detection_dir = setup_directories(base_path)
    
    cap = cv2.VideoCapture(stream_url if stream_url.startswith("http") else 0)
    frame_id = 0

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Perform object detection
        results = model(frame)
        detected_objects = results.pandas().xyxy[0].to_dict(orient="records")
        
        # Save the current frame image in /images folder
        frame_image_path = image_dir / f'frame_{frame_id:04d}.jpg'
        cv2.imwrite(str(frame_image_path), frame)
        
        distance = ultrasonic_sensor.get_distance_at(0)  # 0 degrees is straight ahead
        
        save_detection_data(detection_dir, frame_id, detected_objects, distance)

        frame_id += 1

    cap.release()


def camera_and_ultrasonic_data_collection():
    stream_url = '0'  # Use 0 for the default camera, modify as needed for other sources
    model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
    ultrasonic_sensor = Ultrasonic(trig=Pin("D8"), echo=Pin("D9"))
    process_stream(stream_url, model, ultrasonic_sensor)

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

    # Start the camera and ultrasonic sensor data collection in a separate thread
    data_collection_thread = threading.Thread(target=camera_and_ultrasonic_data_collection, daemon=True)
    data_collection_thread.start()

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
                    robot.turn_left(100)
                elif char == curses.KEY_RIGHT or char == ord('d'):
                    robot.turn_right(100)
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
