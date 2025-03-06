#Goal
# The AGV needs to predict where objects (e.g., people, boxes, other AGVs) are moving. 
# Instead of treating every frame as a fresh detection, tracking movement over time helps with:

    #1. Collision avoidance (knowing if something is on a collision course).
    #2. Path planning (adjusting speed/direction based on moving obstacles).

#It also helps in debugging. Before deploying an AGV, we need to visualize what it sees.
#If tracking is failing in a cluttered environment, seeing movement paths helps us tune DeepSORT better.

import cv2
import torch
import supervision as sv
from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort

# Load YOLO model
yolo_model = YOLO("../models/yolov8n.pt")

# Initialize DeepSORT tracker
deep_sort = DeepSort(max_age=30)  # Increase max_age to help with re-identification

# Store object paths
object_paths = {}

# Open video capture
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
        detections.append(([x1, y1, x2, y2], score, class_id))
    
    # Update DeepSORT tracker
    tracks = deep_sort.update_tracks(detections, frame=frame)
    
    for track in tracks:
        if not track.is_confirmed():
            continue

        track_id = track.track_id
        x1, y1, x2, y2 = track.to_ltrb()
        centroid = (int((x1 + x2) / 2), int((y1 + y2) / 2))

        # Store path history
        if track_id not in object_paths:
            object_paths[track_id] = []
        object_paths[track_id].append(centroid)

        # Draw bounding box
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        cv2.putText(frame, f'ID {track_id}', (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Draw movement paths
    for track_id, path in object_paths.items():
        for i in range(1, len(path)):
            cv2.line(frame, path[i - 1], path[i], (255, 0, 0), 2)

    cv2.imshow("Tracking with Paths", frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
