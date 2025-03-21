#!/usr/bin/env python3
import cv2
import numpy as np
import serial
import time
import RPi.GPIO as GPIO
from ultralytics import YOLO

# ----- Serial Communication Setup -----
# Update the serial port as needed (commonly /dev/ttyACM0 or /dev/ttyUSB0 on the Pi)
arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)  # Allow time for the Arduino to reset

# ----- Ultrasonic Sensor Setup (HC-SR04) -----
TRIG = 23
ECHO = 24
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def get_distance():
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    pulse_start = time.time()
    pulse_end = time.time()
    # Wait for the pulse to start
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
    # Wait for the pulse to end
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()
    pulse_duration = pulse_end - pulse_start
    # Calculate distance (cm)
    distance = pulse_duration * 17150
    return round(distance, 2)

# ----- YOLO Model Setup for Dog Recognition -----
# Replace "best.pt" with the path to your trained model.
model = YOLO("best.pt")
# We assume that class "0" represents a dog.

# ----- USB Camera Setup -----
cap = cv2.VideoCapture(0)  # Access the USB camera
if not cap.isOpened():
    print("Error: Could not open USB camera.")
    exit()

searching = False
last_command = None  # Used to avoid sending redundant commands

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    height, width, _ = frame.shape

    # ----- YOLO Object Detection -----
    results = model(frame)
    dog_detected = False
    center_x = None

    # Loop through detections (assuming each result contains 'boxes' with xyxy, confidence, and class)
    for result in results:
        for box, conf, cls in zip(result.boxes.xyxy, result.boxes.conf, result.boxes.cls):
            if conf.item() >= 0.7:
                x1, y1, x2, y2 = map(int, box[:4])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                center_x = (x1 + x2) // 2
                # Assuming class label 0 corresponds to a dog
                if int(cls.item()) == 0:
                    dog_detected = True
                    searching = False
                    break
        if dog_detected:
            break

    # ----- Ultrasonic Distance Reading -----
    distance = get_distance()
    print(f"Distance: {distance} cm")

    # ----- Determine Movement Command Based on Dog Detection & Distance -----
    if dog_detected:
        # If the dog is detected, use its horizontal position to decide turning.
        if center_x is not None:
            if center_x < width // 3:
                command = "LEFT"
            elif center_x > 2 * width // 3:
                command = "RIGHT"
            else:
                # When centered, use distance to decide whether to move forward or stop.
                if distance > 50:
                    command = "FORWARD"
                elif distance < 20:
                    command = "STOP"
                else:
                    command = "STOP"
        else:
            command = "STOP"
    else:
        # No dog detected: initiate a rotating search pattern.
        if not searching:
            command = "TURN"
            searching = True
        else:
            command = "SLOW_TURN"

    # ----- Send Command to Arduino via Serial (only if it changes) -----
    if command != last_command:
        arduino.write((command + "\n").encode())
        print(f"Command sent: {command}")
        last_command = command
        time.sleep(0.1)  # Short delay to avoid flooding with commands

    # ----- Display the Camera Frame for Debugging -----
    cv2.imshow("Frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup before exit
cap.release()
cv2.destroyAllWindows()
GPIO.cleanup()
arduino.close()