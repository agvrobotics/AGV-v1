#Goal
# Defines Zones – Path, Pickup, and Danger zones using polygon coordinates.
# Overlays a Grid – Draws these zones on the screen for visualization.
# Classifies Detections – Checks if detected objects (people, AGVs, boxes, etc.) are inside a zone.
# Highlights Relevant Objects – Assigns a color & label to objects based on which zone they're in.
import cv2
import numpy as np
import torch
import supervision as sv
from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort

# 🔵 Aqua (255, 255, 0) → Path Zone (Objects detected on the AGV’s path)
# 🟢 Green (0, 255, 0) → Pickup Zone (Objects at the pickup point)
# 🔴 Red (0, 0, 255) → Danger Zone (Humans, obstacles—things the AGV should avoid)
# ⚪ White (255, 255, 255) → Outside All Zones (Objects that aren’t in any relevant AGV area)
# Define zone coordinates (adjust as needed)
PATH_ZONE = [(100, 300), (500, 300), (500, 500), (100, 500)]
PICKUP_ZONE = [(600, 300), (800, 300), (800, 500), (600, 500)]
DANGER_ZONE = [(200, 100), (700, 100), (700, 250), (200, 250)]

#How Does the Script Identify Objects?
#YOLO Detection Model (e.g., yolov8n.pt) YOLO detects classes like humans, boxes, AGVs, etc.
#It assigns each detection an ID & label (e.g., person = class 0, car = class 2, etc.).

#If YOLO detects an object labeled as "box", "package", "pallet", etc., and it falls inside the Pickup Zone, we classify it as a pickup item.
#If YOLO detects class 0 ("person") inside the Danger Zone, we classify it as a dangerous obstacle.
#If YOLO detects "car", "forklift", "table", etc., and it is in the Path Zone or Danger Zone, we classify it as an obstacle.

# Function to check if a point is inside a polygon
def is_inside_zone(point, zone):
    return cv2.pointPolygonTest(np.array(zone, np.int32), point, False) >= 0

# Load YOLO model
yolo_model = YOLO("../models/yolov8n.pt")

# Initialize DeepSORT tracker
deep_sort = DeepSort(max_age=30)  # Helps re-identification

# Open video capture (change source if needed)
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
        detections.append(([x1, y1, x2 - x1, y2 - y1], score, class_id))

    # Update DeepSORT tracker
    tracks = deep_sort.update_tracks(detections, frame=frame)

    # Draw predefined zones
    cv2.polylines(frame, [np.array(PATH_ZONE, np.int32)], True, (255, 255, 0), 2)
    cv2.polylines(frame, [np.array(PICKUP_ZONE, np.int32)], True, (0, 255, 0), 2)
    cv2.polylines(frame, [np.array(DANGER_ZONE, np.int32)], True, (0, 0, 255), 2)

    for track in tracks:
        if not track.is_confirmed():
            continue

        track_id = track.track_id
        x1, y1, x2, y2 = track.to_ltrb()
        center = (int((x1 + x2) / 2), int((y1 + y2) / 2))  # Compute centroid

        # Assign objects to zones
        if is_inside_zone(center, PATH_ZONE):
            zone_label, color = "PATH", (255, 255, 0)
        elif is_inside_zone(center, PICKUP_ZONE):
            zone_label, color = "PICKUP", (0, 255, 0)
        elif is_inside_zone(center, DANGER_ZONE):
            zone_label, color = "DANGER", (0, 0, 255)
        else:
            zone_label, color = None, (255, 255, 255)

        # Draw detection & zone label
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
        if zone_label:
            cv2.putText(frame, f"{zone_label} {track_id}", (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    cv2.imshow("Region-Based Tracking", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
