#Goal
#Measures time difference between frames to calculate FPS.
#Displays FPS on the top-left corner frame.
#Helps measure how well YOLO is running in real-time.

#FPS Guidelines
#30+ FPS - Great – Real-time detection, smooth processing.
#15 - 30 FPS - Acceptable – Slight delay, but still good.
#5 - 15 FPS	 - Slow – Noticeable lag, needs optimization.
#< 5 FPS - Too slow – YOLO is struggling, needs fixing.

import cv2
import time 
from ultralytics import YOLO

model = YOLO("../models/yolov8n.pt")

# Initialize FPS variables
prev_time = 0

cap = cv2.VideoCapture('/dev/video2')

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)  # Flip horizontally

    # Start timing
    curr_time = time.time()
    fps = 1 / (curr_time - prev_time)  # Calculate FPS
    prev_time = curr_time  # Update previous time

    # Run YOLOv8 object detection
    results = model(frame)

    # Draw detections on the frame
    for result in results:
        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])  # Get bounding box coordinates
            conf = box.conf[0].item()  # Confidence score
            cls = int(box.cls[0].item())  # Class index
            label = f"{model.names[cls]}: {conf:.2f}"

            # Draw rectangle and label
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Display FPS
    cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    # Show the frame
    cv2.imshow("YOLOv8 Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
