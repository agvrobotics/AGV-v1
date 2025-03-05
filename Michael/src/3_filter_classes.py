#Goal
# only detect specific classes.

import cv2
import time 
from ultralytics import YOLO

model = YOLO("../models/yolov8n.pt")

# Initialize FPS variables
prev_time = 0

cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1) 
    curr_time = time.time()
    fps = 1 / (curr_time - prev_time)
    prev_time = curr_time 
    results = model(frame)

    TARGET_CLASSES = ["person", "backpack", "bottle", "cup", "chair", "car", "truck"]

    for result in results:
        for box in result.boxes:
            cls = int(box.cls[0].item())  # Class index
            class_name = model.names[cls]  # Get class name
            
            if class_name in TARGET_CLASSES:  # Only process selected classes
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # Get bounding box coordinates
                conf = box.conf[0].item()  # Confidence score
                label = f"{class_name}: {conf:.2f}"

                # Draw rectangle and label
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


    cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    cv2.imshow("YOLOv8 Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
