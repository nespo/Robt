import cv2
import torch
import json
from pathlib import Path
from datetime import datetime
import numpy as np

def setup_directories(base_path: Path) -> tuple:
    """
    Ensure the directories for storing images and detection data exist.
    """
    image_dir = base_path / 'images'
    detection_dir = base_path / 'detections'
    image_dir.mkdir(parents=True, exist_ok=True)
    detection_dir.mkdir(parents=True, exist_ok=True)
    return image_dir, detection_dir

def save_detection_data(detection_dir: Path, frame_id: int, detected_objects: list):
    """
    Save detection data to a JSON file.
    """
    detection_info = {
        'frame_id': frame_id,
        'timestamp': datetime.now().strftime("%Y%m%d_%H%M%S%f"),
        'detections': detected_objects
    }
    detections_json_path = detection_dir / f'detections_{frame_id:04d}.json'
    with detections_json_path.open('w') as f:
        json.dump(detection_info, f)

def save_frame_image(image_dir: Path, frame_id: int, frame):
    """
    Save the current frame as an image file.
    """
    frame_image_path = image_dir / f'frame_{frame_id:04d}.jpg'
    cv2.imwrite(str(frame_image_path), frame)

def process_stream(stream_url: str, model):
    """
    Continuously process video stream for object detection and save detected objects and frames.
    """
    base_path = Path(__file__).resolve().parent.parent
    image_dir, detection_dir = setup_directories(base_path)

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

        # Perform object detection
        results = model(frame)
        detected_objects = results.pandas().xyxy[0].to_dict(orient="records")
        
        save_detection_data(detection_dir, frame_id, detected_objects)
        save_frame_image(image_dir, frame_id, frame)

        frame_id += 1

    cap.release()

def main(stream_url: str):
    """
    Main function to initialize the model and start the video stream processing.
    """
    model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
    process_stream(stream_url, model)

if __name__ == "__main__":
    stream_url = '0'  # Use 0 for local camera or replace with the appropriate stream URL
    main(stream_url)
