
#!/usr/bin/env python3
import cv2 as cv
import numpy as np
import serial
import time
from ultralytics import YOLO

# ----- Serial Communication Setup -----
# Use the appropriate port for your Arduino on the Pi (commonly /dev/ttyACM0)
arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)  # Allow time for the Arduino to reset

# ----- YOLO Model Setup for Dog Recognition -----
# For a pretrained model, you can use one of the official YOLOv8 files (for example, "yolov8n.pt").
# Alternatively, use your custom model file such as "best.pt". Ensure the model is in the working directory.
model = YOLO("best.pt")
# The ultralytics model will provide a names dictionary (model.names) with class labels.
# COCO-trained YOLOv8 models include "dog" as one of the labels.

# ----- USB Camera Setup -----
import cv2 as cv

camera = cv.VideoCapture(0)
width = 640
height = 480
camera.set (cv.camera_PROP_FRAME_WIDTH, width) 
camera.set(cv.camera_PROP_FRAME_HEIGHT, height)

while True:
    result, frame = camera.read()  # Read frame from the camera
    if not result:
        break
    
    cv.imshow("USB Camera Test", frame)

    if cv.waitKey(1) & 0xFF == ord("q"):
        break

camera.release()
cv.destroyAllWindows()

searching = False
last_command = None  # Store the last sent command to avoid redundant commands

while True:
    ret, frame = camera.read()
    if not ret:
        print("Failed to grab frame")
        break

    height, width, _ = frame.shape

    # ----- YOLO Object Detection -----
    results = model(frame)
    dog_detected = False
    center_x = None

    # Loop through detections.
    # For each box, we check if the detection confidence meets the threshold.
    # Then we check if its label is "dog" (using the model’s names dictionary).
    for result in results:
        for box, conf, cls in zip(result.boxes.xyxy, result.boxes.conf, result.boxes.cls):
            if conf.item() >= 0.7:
                x1, y1, x2, y2 = map(int, box[:4])
                cv.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                center_x = (x1 + x2) // 2
                label = model.names[int(cls.item())]
                if label.lower() == "dog":
                    dog_detected = True
                    searching = False
                    break
        if dog_detected:
            break

    # ----- Determine Movement Command Based on Dog Detection -----
    # Since the ultrasonic sensor is handled by the Arduino,
    # here we decide movement only by the dog's horizontal position.
    if dog_detected:
        if center_x is not None:
            if center_x < width // 3:
                command = "LEFT"
            elif center_x > 2 * width // 3:
                command = "RIGHT"
            else:
                command = "FORWARD"
        else:
            command = "STOP"
    else:
        # If no dog is detected, issue a search command.
        if not searching:
            command = "TURN"
            searching = True
        else:
            command = "SLOW_TURN"

    # ----- Send Command to Arduino via Serial (if it has changed) -----
    if command != last_command:
        arduino.write((command + "\n").encode())
        print(f"Command sent: {command}")
        last_command = command
        time.sleep(0.1)  # Brief delay to avoid flooding with commands

    # Display the current frame (for debugging and visual confirmation)
    cv.imshow("Frame", frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup before exit.
camera.release()
cv.destroyAllWindows()
arduino.close()