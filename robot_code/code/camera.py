import torch
import json
from pathlib import Path
from picamera import PiCamera
from time import sleep
from PIL import Image

# Initialize the PiCamera
camera = PiCamera()
camera.resolution = (1920, 1080) # You can adjust this based on your needs

# Path to save the captured image
image_path = Path('/robot_code/images/captured_image.jpg')
# Ensure the output directory exists
image_path.parent.mkdir(parents=True, exist_ok=True)

# Capture an image
print("Capturing image...")
camera.start_preview()
sleep(2)  # Camera warm-up time
camera.capture(str(image_path))
camera.stop_preview()

# Load the YOLOv5 model
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

# Process the captured image
print("Processing image with YOLOv5...")
results = model(image_path)

# Save processed image with detections
output_dir = Path('/robot_code/detections')
output_dir.mkdir(parents=True, exist_ok=True)
results.save(save_dir=output_dir)

# Extract detection data
detected_objects = results.pandas().xyxy[0].to_dict(orient="records")  # Convert detections to dictionary

# Store detection results
detection_info = {
    'image_path': str(image_path),
    'detections': detected_objects
}

# Save detection data to a JSON file for further use
detections_json_path = output_dir / 'detections.json'
with open(detections_json_path, 'w') as f:
    json.dump(detection_info, f)

print(f"Image processed. Detection data saved to {detections_json_path}")

# Optionally, display the image with detections
# Load the image with PIL and display it if desired
processed_image_path = list(output_dir.glob('*.jpg'))[0]  # Adjust based on saved file
with Image.open(processed_image_path) as img:
    img.show()
