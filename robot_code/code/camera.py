import cv2
import torch
import json
from pathlib import Path
from datetime import datetime
import numpy as np

def setup_directories(base_path: Path) -> tuple:
    """
    Ensure the directories for storing images and detection data exist.

    Parameters:
    - base_path: The base directory path of the project.

    Returns:
    - A tuple of Paths for images and detections directories.
    """
    image_dir = base_path / 'images'
    detection_dir = base_path / 'detections'
    image_dir.mkdir(parents=True, exist_ok=True)
    detection_dir.mkdir(parents=True, exist_ok=True)
    return image_dir, detection_dir

def save_detection_data(detection_dir: Path, frame_id: int, detected_objects: list) -> None:
    """
    Save detection data to a JSON file.

    Parameters:
    - detection_dir: The directory to save detection data files.
    - frame_id: The identifier for the current frame.
    - detected_objects: A list of detected objects in the current frame.
    """
    detection_info = {
        'frame_id': frame_id,
        'timestamp': datetime.now().strftime("%Y%m%d_%H%M%S%f"),
        'detections': detected_objects
    }
    detections_json_path = detection_dir / f'detections_{frame_id:04d}.json'
    with detections_json_path.open('w') as f:
        json.dump(detection_info, f)

def process_stream(stream_url: str, model) -> None:
    """
    Continuously process video stream for object detection.

    Parameters:
    - stream_url: The URL for the video stream.
    - model: The loaded YOLOv5 model for object detection.
    """
    base_path = Path(__file__).parent.parent
    _, detection_dir = setup_directories(base_path)

    cap = cv2.VideoCapture(stream_url)
    frame_id = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame from stream.")
            break

        results = model(frame)
        detected_objects = results.pandas().xyxy[0].to_dict(orient="records")
        save_detection_data(detection_dir, frame_id, detected_objects)

        # Optional: Display the frame with detections
        cv2.imshow('YOLOv5 Object Detection', np.squeeze(results.render()))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        frame_id += 1

    cap.release()
    cv2.destroyAllWindows()

def main(stream_url: str) -> None:
    """
    Main function to initialize the model and start the video stream processing.

    Parameters:
    - stream_url: The URL for the video stream.
    """
    # Load the YOLOv5 model
    model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

    # Process the video stream
    process_stream(stream_url, model)

if __name__ == "__main__":
    stream_url = 'http://192.168.211.75:8080/video'  # Your IP camera stream URL
    main(stream_url)
