#Goal
# Track objects using Norfair library.
#This has more stable id's. If object disappears, it will keep the same id when it reappears.
#Handles occlusions better. If someone walks in front of the object, it will still keep the same id.
#However if object is completely occluded, it will lose the id.
import cv2
import numpy as np 
from ultralytics import YOLO
from norfair import Detection, Tracker, draw_tracked_objects

# Load YOLO model
model = YOLO("../models/yolov8n.pt")

# Initialize Norfair tracker
tracker = Tracker(distance_function="euclidean", distance_threshold=30)

# Open webcam
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    
    frame = cv2.flip(frame, 1)  # Flip horizontally
    results = model(frame)  # Run YOLO detection
    detections = []
    
    for result in results:
        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            centroid = [(x1 + x2) // 2, (y1 + y2) // 2]
            detections.append(Detection(np.array(centroid)))
    
    # Update tracker
    tracked_objects = tracker.update(detections)
    
    # Draw tracked objects with IDs
    draw_tracked_objects(frame, tracked_objects)
    
    # Show frame
    cv2.imshow("YOLOv8 Tracking IDs", frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
