import cv2
import numpy as np
import serial
import time
import RPi.GPIO as GPIO
from picamera2 import Picamera2
from ultralytics import YOLO

# Setup serial communication with Arduino
arduino = serial.Serial('COM17', 9600, timeout=1)
time.sleep(2)  # Wait for connection

# Setup Ultrasonic sensor pins
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
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
    pulse_end = time.time()
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()
    
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    return round(distance, 2)

# Load YOLO model
model = YOLO("best.pt")  # Replace with your model path

dog_label = "dog"

# Initialize PiCamera2
camera = Picamera2()
default_resolution = (1920, 1080)
camera.configure(camera.create_video_configuration(main={"format": 'XRGB8888', "size": default_resolution}))
camera.start()

searching = False
last_command = None  # Store the last sent command to avoid redundant sending

while True:
    frame = camera.capture_array()
    height, width, _ = frame.shape
    
    # YOLO Object Detection
    results = model(frame)
    
    dog_detected = False
    center_x, center_y = None, None
    
    for result in results:
        for box, conf, cls in zip(result.boxes.xyxy, result.boxes.conf, result.boxes.cls):  
            if conf.item() >= 0.7:  
                x1, y1, x2, y2 = map(int, box[:4])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                class_id = int(cls.item())
                if class_id == 0:  # Assuming 0 is the label for dog
                    dog_detected = True
                    searching = False
                    break
    
    distance = get_distance()
    print(f"Distance: {distance} cm")
    
    if dog_detected:
        if center_x < width // 3:
            command = "LEFT"
        elif center_x > 2 * width // 3:
            command = "RIGHT"
        elif distance > 50:
            command = "FORWARD"
        elif distance < 20:
            command = "STOP"
        else:
            command = "STOP"
    else:
        if not searching:
            command = "TURN"
            searching = True
        else:
            command = "SLOW_TURN"

    # Send command only if it has changed
    if command != last_command:
        arduino.write((command + "\n").encode())
        print(f"Command sent: {command}")
        last_command = command  # Update last command
        time.sleep(0.1)  # Add delay to avoid excessive command sending
    
    # Display output
    cv2.imshow("Frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
GPIO.cleanup()