import cv2
import torch
import json
import subprocess
from pathlib import Path
from datetime import datetime
import numpy as np

def setup_directories(base_path: Path) -> tuple:
    image_dir = base_path / 'images'
    detection_dir = base_path / 'detections'
    image_dir.mkdir(parents=True, exist_ok=True)
    detection_dir.mkdir(parents=True, exist_ok=True)
    return image_dir, detection_dir

def save_detection_data(detection_dir: Path, frame_id: int, detected_objects: list):
    detection_info = {
        'frame_id': frame_id,
        'timestamp': datetime.now().strftime("%Y%m%d_%H%M%S%f"),
        'detections': detected_objects
    }
    detections_json_path = detection_dir / f'detections_{frame_id:04d}.json'
    with detections_json_path.open('w') as f:
        json.dump(detection_info, f)

def capture_frame_with_rpicam(output_path):
    # Replace 'rpicam-still' with 'rpicam-jpeg' if needed
    capture_command = ['rpicam-still', '-o', str(output_path)]
    subprocess.run(capture_command, check=True)

def process_frame(image_path, model, detection_dir, frame_id):
    frame = cv2.imread(str(image_path))
    if frame is not None:
        # Perform object detection
        results = model(frame)
        detected_objects = results.pandas().xyxy[0].to_dict(orient="records")
        save_detection_data(detection_dir, frame_id, detected_objects)

def main():
    base_path = Path(__file__).resolve().parent.parent
    image_dir, detection_dir = setup_directories(base_path)
    model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

    frame_id = 0
    while True:
        image_path = image_dir / f'frame_{frame_id:04d}.jpg'
        capture_frame_with_rpicam(image_path)
        process_frame(image_path, model, detection_dir, frame_id)
        frame_id += 1
        # Add any desired delay or breaking condition here

if __name__ == "__main__":
    main()
