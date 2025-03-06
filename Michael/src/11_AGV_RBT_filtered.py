#Goal
#Dynamically Adjusts Zones Based on AGV Position – Instead of using static coordinates, the script recalculates zone boundaries in real-time as the AGV moves, ensuring the detection areas stay relevant.
#Filters Out Objects Outside Relevant Zones – Objects detected outside the dynamically adjusted Path, Pickup, or Danger zones are completely ignored, improving processing efficiency and reducing unnecessary tracking.
#Maintains Real-Time Visualization of Adaptive Zones – The script continuously overlays updated zone boundaries on the video feed, providing a clear visual representation of where the AGV is focusing its attention.
import cv2
import numpy as np
from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort

# Initialize YOLO & DeepSORT
yolo_model = YOLO("../models/yolov8n.pt")
tracker = DeepSort(max_age=30)

# Object class IDs
HUMAN_CLASS_ID = 0  # "person"
BOX_CLASSES = {24, 25, 26}  # "suitcase", "backpack", "handbag"
OBSTACLE_CLASSES = {2, 3, 7}  # "car", "truck", "forklift"

# AGV Position (for dynamic zones)
agv_x, agv_y = 300, 400

# Define dynamic zones based on AGV position
def update_zones(agv_x, agv_y):
    path_offset = 150
    pickup_offset = 250
    danger_offset = 100
    
    path_zone = [(agv_x - path_offset, agv_y + 100), (agv_x + path_offset, agv_y + 100),
                 (agv_x + path_offset, agv_y + 250), (agv_x - path_offset, agv_y + 250)]
    
    pickup_zone = [(agv_x + pickup_offset, agv_y), (agv_x + pickup_offset + 100, agv_y),
                   (agv_x + pickup_offset + 100, agv_y + 150), (agv_x + pickup_offset, agv_y + 150)]
    
    danger_zone = [(agv_x - danger_offset, agv_y - 100), (agv_x + danger_offset, agv_y - 100),
                   (agv_x + danger_offset, agv_y + 50), (agv_x - danger_offset, agv_y + 50)]
    
    return path_zone, pickup_zone, danger_zone

# Function to check if a point is inside a zone
def is_inside_zone(point, zone):
    return cv2.pointPolygonTest(np.array(zone, np.int32), point, False) >= 0

# Start video capture
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    
    # Update dynamic zones based on AGV's position
    PATH_ZONE, PICKUP_ZONE, DANGER_ZONE = update_zones(agv_x, agv_y)
    
    # Run YOLO detection
    results = yolo_model(frame)[0]
    detections = []
    
    for det in results.boxes.data.tolist():
        x1, y1, x2, y2, conf, class_id = det
        class_id = int(class_id)
        
        # Filter only required object classes
        if class_id not in {HUMAN_CLASS_ID} | BOX_CLASSES | OBSTACLE_CLASSES:
            continue  # Ignore irrelevant objects
        
        detections.append(([x1, y1, x2, y2], conf, class_id))
    
    # Track objects
    tracks = tracker.update_tracks(detections, frame=frame)
    
    for track in tracks:
        if not track.is_confirmed():
            continue
        
        track_id = track.track_id
        x1, y1, x2, y2 = track.to_ltwh()  # Convert to (x1, y1, x2, y2)
        center = (int((x1 + x2) / 2), int((y1 + y2) / 2))
        class_id = track.det_class
        
        # Determine object type
        if class_id == HUMAN_CLASS_ID:
            obj_type = "Human"
            color = (0, 0, 255)  # Red (Danger)
        elif class_id in BOX_CLASSES:
            obj_type = "Pickup Item"
            color = (0, 255, 0)  # Green (Pickup)
        elif class_id in OBSTACLE_CLASSES:
            obj_type = "Obstacle"
            color = (255, 0, 0)  # Blue (Obstacle)
        else:
            continue  # Ignore objects outside predefined classes
        
        # Assign objects to zones
        if is_inside_zone(center, PATH_ZONE):
            zone_label = "PATH"
        elif is_inside_zone(center, PICKUP_ZONE):
            zone_label = "PICKUP"
        elif is_inside_zone(center, DANGER_ZONE):
            zone_label = "DANGER"
        else:
            continue  # Fully ignore detections outside relevant zones
        
        # Draw bounding box & label
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
        cv2.putText(frame, f"{zone_label} {obj_type} {track_id}", (int(x1), int(y1) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    
    # Draw zones
    cv2.polylines(frame, [np.array(PATH_ZONE, np.int32)], True, (255, 255, 0), 2)
    cv2.polylines(frame, [np.array(PICKUP_ZONE, np.int32)], True, (0, 255, 0), 2)
    cv2.polylines(frame, [np.array(DANGER_ZONE, np.int32)], True, (0, 0, 255), 2)
    
    cv2.imshow("AGV Region-Based Tracking", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
