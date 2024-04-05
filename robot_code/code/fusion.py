import cv2
import torch
import json
from pathlib import Path
from datetime import datetime
import numpy as np
# Assuming the Ultrasonic, Servo, PWM, and Pin classes are in the modules directory relative to this script
from modules.ultrasonic import Ultrasonic
from modules.servo import Servo
from modules.pwm import PWM
from modules.pin import Pin

def setup_directories(base_path: Path) -> tuple:
    """
    Ensure the directories for storing images and detection data exist.
    """
    image_dir = base_path / 'images'
    detection_dir = base_path / 'detections'
    image_dir.mkdir(parents=True, exist_ok=True)
    detection_dir.mkdir(parents=True, exist_ok=True)
    return image_dir, detection_dir

def save_detection_data(detection_dir: Path, frame_id: int, detected_objects: list, distance: float) -> None:
    """
    Save detection data to a JSON file including ultrasonic sensor distance.
    """
    detection_info = {
        'frame_id': frame_id,
        'timestamp': datetime.now().strftime("%Y%m%d_%H%M%S%f"),
        'detections': detected_objects,
        'distance': distance
    }
    detections_json_path = detection_dir / f'detections_{frame_id:04d}.json'
    with detections_json_path.open('w') as f:
        json.dump(detection_info, f)

def process_stream(stream_url: str, model, ultrasonic_sensor) -> None:
    base_path = Path(__file__).resolve().parent.parent
    _, detection_dir = setup_directories(base_path)
    
    cap = cv2.VideoCapture(stream_url if stream_url.startswith("http") else 0)
    frame_id = 0

    if not cap.isOpened():
        print("Error: Could not open video source.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame from stream.")
            break

        results = model(frame)
        detected_objects = results.pandas().xyxy[0].to_dict(orient="records")
        
        # Get distance from the ultrasonic sensor at a specific angle, assuming forward facing
        distance = ultrasonic_sensor.get_distance_at(0)  # 0 degrees is straight ahead

        save_detection_data(detection_dir, frame_id, detected_objects, distance)

        frame_id += 1

    cap.release()

def main(stream_url: str) -> None:
    model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

    # Initialize your ultrasonic sensor here with the appropriate pins
    ultrasonic_sensor = Ultrasonic(trig=Pin("GPIO_PIN_FOR_TRIG"), echo=Pin("GPIO_PIN_FOR_ECHO"))

    process_stream(stream_url, model, ultrasonic_sensor)

if __name__ == "__main__":
    stream_url = '0'  # Use 0 for the default camera, modify as needed for other sources
    main(stream_url)
