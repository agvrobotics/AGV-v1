import cv2
import numpy as np

# Load YOLO model
net = cv2.dnn.readNetFromONNX("../models/yolov8n.onnx")

# Load class names (COCO dataset)
class_names = ["person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
               "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
               "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
               "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
               "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
               "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
               "potted plant", "bed", "dining table", "toilet", "TV", "laptop", "mouse", "remote", "keyboard",
               "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
               "scissors", "teddy bear", "hair drier", "toothbrush"]

# Open webcam
cap = cv2.VideoCapture(0)  # Change to 1 if using an external webcam

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Convert frame to blob (for YOLO)
    blob = cv2.dnn.blobFromImage(frame, scalefactor=1/255.0, size=(640, 640), swapRB=True, crop=False)
    net.setInput(blob)

    # Run YOLO detection
    detections = net.forward()

    # Process detections
    for detection in detections[0]:
        confidence = detection[4]
        if confidence > 0.5:  # Confidence threshold
            class_id = np.argmax(detection[5:])
            class_name = class_names[class_id]
            
            # Bounding box (scaled to frame size)
            cx, cy, w, h = detection[:4] * np.array([frame.shape[1], frame.shape[0], frame.shape[1], frame.shape[0]])
            x, y = int(cx - w / 2), int(cy - h / 2)
            w, h = int(w), int(h)

            # Draw bounding box
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, class_name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Show frame
    cv2.imshow("YOLO Object Detection", frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
