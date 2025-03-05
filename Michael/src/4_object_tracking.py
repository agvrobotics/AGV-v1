#Goal
# Track objects using Norfair library.
#Yolo detects objects and Norfair tracks them.
#This prevents object from disappearing when they move.
import cv2
import numpy as np
from ultralytics import YOLO
from norfair import Tracker, Detection

# Load YOLOv8 model
model = YOLO("../models/yolov8n.pt")

# Open webcam (0 = default laptop camera, change to 1 for external webcam)
cap = cv2.VideoCapture(0)

# Initialize Norfair tracker
tracker = Tracker(distance_function="euclidean", distance_threshold=30)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)  # Flip horizontally

    # Run YOLOv8 object detection
    results = model(frame)

    detections = []
    for result in results:
        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])  # Get bounding box coordinates
            conf = box.conf[0].item()  # Confidence score
            cls = int(box.cls[0].item())  # Class index
            label = f"{model.names[cls]}: {conf:.2f}"

            # Create detection object for tracking
            centroid = [(x1 + x2) / 2, (y1 + y2) / 2]
            detections.append(Detection(np.array(centroid)))

            # Draw bounding box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Update tracker with new detections
    tracked_objects = tracker.update(detections)

    # Draw tracking IDs
    for obj in tracked_objects:
        x, y = map(int, obj.estimate[0])
        cv2.putText(frame, f"ID {obj.id}", (x, y - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)

    # Show frame
    cv2.imshow("YOLOv8 Object Tracking", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
