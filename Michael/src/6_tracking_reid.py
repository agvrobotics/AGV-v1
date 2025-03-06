#In 5_tracking_id, we relied only on motion (centroid tracking), which can lose track if an object is briefly occluded. 
#Now, we add ReID, so if an object disappears and reappears, it gets the same ID.

import cv2
import torch
import numpy as np
from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort

# Load YOLOv8 model
yolo_model = YOLO("../models/yolov8n.pt")

# Initialize DeepSORT tracker
tracker = DeepSort(max_age=30)

cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Run YOLO detection
    results = yolo_model(frame)[0]
    detections = []

    for r in results.boxes.data.tolist():
        x1, y1, x2, y2, score, class_id = r
        if score > 0.5:  # Confidence threshold
            detections.append(([x1, y1, x2, y2], score, int(class_id)))

    # Update tracker
    tracked_objects = tracker.update_tracks(detections, frame=frame)

    # Draw bounding boxes and IDs
    for track in tracked_objects:
        if not track.is_confirmed():
            continue
        track_id = track.track_id
        x1, y1, x2, y2 = map(int, track.to_tlbr())
        label = f"ID {track_id}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # Show frame
    cv2.imshow("YOLO + DeepSORT Tracking", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
