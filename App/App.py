'''
Blynk integration using a Button widget template:
    V0: Switch control
    V1: Switch value
    V2: Seconds (treat timer)
    V3: Button image
Live video streaming using Flask
'''

import cv2
import serial
import time
import threading
import numpy as np
import pyaudio
import RPi.GPIO as GPIO
import BlynkLib
from flask import Flask, Response


# Hardware and Serial Setup
GPIO.setmode(GPIO.BCM)
SOLENOID_PIN = 17  # BCM pin 17
GPIO.setup(SOLENOID_PIN, GPIO.OUT)
GPIO.output(SOLENOID_PIN, GPIO.HIGH)

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
ser.flush()


# Object Detection Setup
classNames = []
classFile = "/home/eg1004/Desktop/PawPal/Object_Detection_Files/coco.names"
with open(classFile, "rt") as f:
    classNames = f.read().rstrip("\n").split("\n")

configPath = "/home/eg1004/Desktop/PawPal/Object_Detection_Files/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
weightsPath = "/home/eg1004/Desktop/PawPal/Object_Detection_Files/frozen_inference_graph.pb"

net = cv2.dnn_DetectionModel(weightsPath, configPath)
net.setInputSize(320, 320)
net.setInputScale(1.0 / 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

def getObjects(img, thres, nms, draw=True, objects=[]):
    classIds, confs, bbox = net.detect(img, confThreshold=thres, nmsThreshold=nms)
    objectInfo = []
    if len(objects) == 0:
        objects = classNames
    if len(classIds) != 0:
        for classId, confidence, box in zip(classIds.flatten(), confs.flatten(), bbox):
            className = classNames[classId - 1]
            if className in objects:
                objectInfo.append([box, className])
                if draw:
                    cv2.rectangle(img, box, color=(0, 255, 0), thickness=2)
                    cv2.putText(img, className.upper(), (box[0] + 10, box[1] + 30),
                                cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(img, str(round(confidence * 100, 2)), (box[0] + 200, box[1] + 30),
                                cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
    return img, objectInfo


# Bark Detection Setup (Audio)
def detect_bark():
    CHUNK = 1024
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 48000
    threshold = 15000
    
    p = pyaudio.PyAudio()
    device_index = 2

    try:
        stream = p.open(format=FORMAT,
                        channels=CHANNELS,
                        rate=RATE,
                        input=True,
                        frames_per_buffer=CHUNK,
                        input_device_index=device_index)
    except Exception:
        p.terminate()
        return

    while True:
        try:
            data = stream.read(CHUNK, exception_on_overflow=False)
            audio_data = np.frombuffer(data, dtype=np.int16)
            rms = np.sqrt(np.mean(np.square(audio_data)))
            if rms > threshold:
                ser.write(b"BARK\n")
                time.sleep(1)
        except Exception:
            continue

    stream.stop_stream()
    stream.close()
    p.terminate()


# Solenoid Control (Treat Dispensing)
def solenoid_control():
    treat_interval = 10
    while True:
        time.sleep(treat_interval)
        GPIO.output(SOLENOID_PIN, GPIO.LOW)
        time.sleep(1)
        GPIO.output(SOLENOID_PIN, GPIO.HIGH)
        ser.write(b"TREAT\n")
        device_config['last_treat_dispensed'] = time.strftime("%Y-%m-%d %H:%M:%S")


# Blynk Integration Setup
BLYNK_AUTH = "h5u5Jzov-zcEvMCYGTVdzjE8EBx_0-2O"  
blynk = BlynkLib.Blynk(BLYNK_AUTH)

device_config = {
    'treat_interval': 10,
    'switch_value': 0,
    'last_treat_dispensed': "Never"
}

@blynk.handle_event('write V0')
def handle_switch_control(pin, value):
    print("Switch control (V0) command:", value)

@blynk.handle_event('write V1')
def handle_switch_value(pin, value):
    try:
        device_config['switch_value'] = int(value[0])
        print("Switch value (V1) updated to:", device_config['switch_value'])
        ser.write(f"SW:{device_config['switch_value']}\n".encode())
    except Exception as e:
        print("Error updating switch value:", e)

@blynk.handle_event('write V2')
def handle_seconds(pin, value):
    try:
        new_seconds = int(value[0])
        device_config['treat_interval'] = new_seconds
        print("Treat timer (V2) updated to:", new_seconds, "seconds")
        ser.write(f"TIMER:{new_seconds}\n".encode())
    except Exception as e:
        print("Error updating treat timer seconds:", e)

@blynk.handle_event('write V3')
def handle_button_image(pin, value):
    print("Button image state (V3) received:", value)

def update_last_treat():
    while True:
        blynk.virtual_write(3, "Last treat: " + device_config['last_treat_dispensed'])
        time.sleep(5)

def update_switch_value():
    while True:
        blynk.virtual_write(1, device_config['switch_value'])
        time.sleep(5)

def run_blynk():
    while True:
        blynk.run()


# Flask-based Live Video Streaming Setup
app = Flask(__name__)
cap_flask = cv2.VideoCapture(0)
cap_flask.set(3, 640)
cap_flask.set(4, 480)

def generate_frames():
    while True:
        success, frame = cap_flask.read()
        if not success:
            break
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

def run_flask():
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)


# Main Application
if __name__ == "__main__":
    # Start Blynk thread
    blynk_thread = threading.Thread(target=run_blynk, daemon=True)
    blynk_thread.start()

    # Start threads to update Blynk data streams
    last_treat_thread = threading.Thread(target=update_last_treat, daemon=True)
    last_treat_thread.start()
    switch_value_thread = threading.Thread(target=update_switch_value, daemon=True)
    switch_value_thread.start()

    # Start bark detection and solenoid control threads
    bark_thread = threading.Thread(target=detect_bark, daemon=True)
    bark_thread.start()
    solenoid_thread = threading.Thread(target=solenoid_control, daemon=True)
    solenoid_thread.start()

    # Start Flask thread for live video streaming
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()

    # Set up video capture for object detection in the main loop
    cap = cv2.VideoCapture(0)
    cap.set(3, 640)
    cap.set(4, 480)
    
    last_rotation_time = time.time()
    rotation_interval = 30

    while True:
        success, img = cap.read()
        if not success:
            continue
        img, objectInfo = getObjects(img, 0.45, 0.2, objects=['dog'])
        if objectInfo:
            ser.write(b"FOLLOW\n")
        else:
            current_time = time.time()
            if current_time - last_rotation_time > rotation_interval:
                ser.write(b"ROTATE\n")
                last_rotation_time = current_time
            else:
                ser.write(b"STOP\n")
        cv2.imshow("Output", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    ser.close()
