import cv2
import torch
import supervision as sv
from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort

# Load YOLO model
yolo_model = YOLO("../models/yolov8n.pt")

# Initialize DeepSORT tracker with tuned parameters
tracker = DeepSort(
    max_age=30,  # Increase persistence (keeps IDs longer when occluded)
    embedder="mobilenet",
    embedder_gpu=True if torch.cuda.is_available() else False,
    max_iou_distance=0.7,  # Adjust matching threshold
    n_init=3,  # Reduce frames needed before assigning an ID
)

# Open webcam
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    
    results = yolo_model(frame, stream=True)
    detections = []
    
    for result in results:
        for box in result.boxes.xyxy:
            x1, y1, x2, y2 = map(int, box)
            detections.append(([x1, y1, x2, y2], 0.8, 'object'))  # Adjust confidence threshold
    
    # Update tracker
    tracked_objects = tracker.update_tracks(detections, frame=frame)
    
    # Draw results
    for track in tracked_objects:
        if not track.is_confirmed():
            continue
        x1, y1, x2, y2 = map(int, track.to_ltrb())
        track_id = track.track_id
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f'ID: {track_id}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    cv2.imshow("Tracking", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
