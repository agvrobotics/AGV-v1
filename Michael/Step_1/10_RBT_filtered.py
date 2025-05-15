#Goal
#Tracks & Classifies Objects in Zones – Uses YOLO for object detection and DeepSORT for tracking, then assigns objects to predefined zones (Path, Pickup, Danger) based on their centroids.
#Filters & Labels Objects – Identifies object types (e.g., person, box) using YOLO class IDs, then filters out irrelevant detections while labeling important ones with appropriate colors.
#Real-Time Visualization for AGV Decision-Making – Continuously updates the AGV’s perception by drawing bounding boxes, movement paths, and zone overlays, ensuring real-time awareness of obstacles and pickups.
import cv2
import numpy as np
from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort

yolo_model = YOLO("../models/yolov8n.pt")

# Initialize DeepSORT Tracker
tracker = DeepSort(max_age=30)

# Define relevant YOLO class IDs (adjust as needed)
HUMAN_CLASS_ID = 0  # "person"
BOX_CLASSES = {24, 25, 26}  # "suitcase" (24), "backpack" (25), and "handbag" (26)
OBSTACLE_CLASSES = {2, 3, 7}  # "car" (2), "truck" (3), and "forklift" (7)
#if an object is detected but not in the predefined lists, it is ignored or logged as "Unknown."

# Define zone coordinates (adjust as needed)
PATH_ZONE = [(100, 300), (500, 300), (500, 500), (100, 500)]
PICKUP_ZONE = [(600, 300), (800, 300), (800, 500), (600, 500)]
DANGER_ZONE = [(200, 100), (700, 100), (700, 250), (200, 250)]

# Function to check bounding box overlap with a zone
# bbox" to refer to detected object boundaries.(Bounding Box)
def get_zone_overlap(bbox, zone):
    x1, y1, x2, y2 = bbox
    bbox_polygon = np.array([(x1, y1), (x2, y1), (x2, y2), (x1, y2)], np.int32)
    zone_polygon = np.array(zone, np.int32)

    mask = np.zeros((1080, 1920), dtype=np.uint8)  # Adjust based on video resolution
    cv2.fillPoly(mask, [bbox_polygon], 255)
    zone_mask = np.zeros_like(mask)
    cv2.fillPoly(zone_mask, [zone_polygon], 255)

    intersection = cv2.bitwise_and(mask, zone_mask)
    overlap_area = np.sum(intersection == 255)

    return overlap_area > 500  # Minimum area threshold to consider overlap

# Start video capture
cap = cv2.VideoCapture('/dev/video2')

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Run YOLO detection
    results = yolo_model(frame)

    detections = []
    for result in results:
        for box in result.boxes.data:
            x1, y1, x2, y2, conf, class_id = box.tolist()
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            
            # Filter out low-confidence detections
            if conf < 0.4:
                continue

            detections.append(([x1, y1, x2 - x1, y2 - y1], conf, int(class_id)))  # (bbox, confidence, class_id)

    # Run DeepSORT tracking
    tracks = tracker.update_tracks(detections, frame=frame)

    # Draw zones
    cv2.polylines(frame, [np.array(PATH_ZONE, np.int32)], True, (255, 255, 0), 2)  # Aqua
    cv2.polylines(frame, [np.array(PICKUP_ZONE, np.int32)], True, (0, 255, 0), 2)  # Green
    cv2.polylines(frame, [np.array(DANGER_ZONE, np.int32)], True, (0, 0, 255), 2)  # Red

    for track in tracks:
        if not track.is_confirmed():
            continue
        
        track_id = track.track_id
        x1, y1, w, h = track.to_ltwh()  # Convert to (x, y, width, height)
        x2, y2 = x1 + w, y1 + h
        bbox = (x1, y1, x2, y2)
        class_id = track.det_class  # YOLO class ID from detection

        # Identify object type
        if class_id == HUMAN_CLASS_ID:
            obj_type = "Human"
        elif class_id in BOX_CLASSES:
            obj_type = "Pickup Item"
        elif class_id in OBSTACLE_CLASSES:
            obj_type = "Obstacle"
        else:
            #if an object is detected but not in the predefined lists, it is ignored or logged as "Unknown."
            obj_type = "Unknown"

        # Assign zones based on overlap
        if get_zone_overlap(bbox, DANGER_ZONE):
            zone_label = "DANGER"
            color = (0, 0, 255)  # Red
        elif get_zone_overlap(bbox, PICKUP_ZONE):
            zone_label = "PICKUP"
            color = (0, 255, 0)  # Green
        elif get_zone_overlap(bbox, PATH_ZONE):
            zone_label = "PATH"
            color = (255, 255, 0)  # Aqua
        else:
            zone_label = "UNKNOWN"
            color = (255, 255, 255)  # White

        # Draw bounding box and label
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
        cv2.putText(frame, f"{zone_label} {obj_type} {track_id}", (int(x1), int(y1) - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)


    cv2.imshow("Region-Based Tracking", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
