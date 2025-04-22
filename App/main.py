'''
Flask-based live video streaming and device control for a Smart Pet Device:
Extra Credit App Features:
  - Displays the last treat dispensed and current distance from the dog
  - Two buttons to set the treat dispensing timer to 15 or 30 seconds
  - Streams live video from the USB camera to App
'''

import cv2
import serial
import time
import threading
import numpy as np
import pyaudio
import RPi.GPIO as GPIO
from flask import Flask, Response, request


# Hardware and Serial Setup
GPIO.setmode(GPIO.BCM)
SOLENOID_PIN = 17  # Use BCM pin 17 for solenoid control
GPIO.setup(SOLENOID_PIN, GPIO.OUT)
GPIO.output(SOLENOID_PIN, GPIO.HIGH)  # Default state: off (active-low)

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
ser.flush()


# Object Detection Setup (using OpenCV DNN)
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


# Global Device Configuration

# This dictionary holds parameters that the web app can update.
device_config = {
    'treat_interval': 10,       # Default treat timer (seconds)
    'last_treat_dispensed': "Never",
    'current_distance': "N/A"
}

# Bark Detection Setup (Using PyAudio)
def detect_bark():
    CHUNK = 1024
    FORMAT = pyaudio.paInt16  # 16-bit audio format
    CHANNELS = 1              # Mono channel
    RATE = 48000              # 48kHz sample rate
    threshold = 15000         # RMS threshold for bark detection

    p = pyaudio.PyAudio()
    device_index = 2 # Change according to assigned channel when doing arecord -l

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
                time.sleep(1)  # Prevent rapid retriggering
        except Exception:
            continue

    stream.stop_stream()
    stream.close()
    p.terminate()


# Solenoid Control (Treat Dispensing)
def solenoid_control():
    while True:
        # Use the treat_interval from the global device_config
        time.sleep(device_config['treat_interval'])
        GPIO.output(SOLENOID_PIN, GPIO.LOW)  # Activate solenoid (active-low)
        time.sleep(1)
        GPIO.output(SOLENOID_PIN, GPIO.HIGH) # Deactivate solenoid
        ser.write(b"TREAT\n")
        device_config['last_treat_dispensed'] = time.strftime("%Y-%m-%d %H:%M:%S")


# Flask App Setup for Live Video Streaming and Control
app = Flask(__name__)

# A separate video capture dedicated for Flask streaming
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
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# Main web page for control and display
@app.route('/')
def index():
    html = '''
    <!doctype html>
    <html>
    <head>
        <title>Smart Pet Device Control</title>
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <!-- You can add a manifest file here for a PWA -->
    </head>
    <body>
        <h1>Smart Pet Device Control</h1>
        <p>Last treat dispensed: {last_treat}</p>
        <p>Current Distance: {current_distance}</p>
        <button onclick="setTimer(15)">Set Timer to 15 Seconds</button>
        <button onclick="setTimer(30)">Set Timer to 30 Seconds</button>
        <h2>Live Video Feed</h2>
        <img src="/video_feed" width="640" height="480">
        <script>
            function setTimer(value){
                fetch('/set_timer?value=' + value)
                .then(response => response.text())
                .then(data => { alert(data); window.location.reload(); });
            }
            // Optionally, register a service worker for PWA functionality.
        </script>
    </body>
    </html>
    '''.format(last_treat=device_config['last_treat_dispensed'],
               current_distance=device_config['current_distance'])
    return html

# Endpoint to update the treat timer via a GET request.
@app.route('/set_timer')
def set_timer():
    value = request.args.get('value', default=10, type=int)
    device_config['treat_interval'] = value
    return "Timer set to {} seconds.".format(value)

def run_flask():
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)


# Main Application Loop (Device Functions
if __name__ == "__main__":
    # Start the Flask server in a separate thread for the web interface and live video
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()

    # Start bark detection and solenoid control threads
    bark_thread = threading.Thread(target=detect_bark, daemon=True)
    bark_thread.start()
    solenoid_thread = threading.Thread(target=solenoid_control, daemon=True)
    solenoid_thread.start()

    # Set up video capture for object detection in the main loop
    cap = cv2.VideoCapture(0)
    cap.set(3, 640)
    cap.set(4, 480)
    
    last_rotation_time = time.time()
    rotation_interval = 30  # seconds

    while True:
        success, img = cap.read()
        if not success:
            continue

        # Run object detection (e.g., to look for a dog)
        img, objectInfo = getObjects(img, 0.45, 0.2, objects=['dog'])
        if objectInfo:
            # Simulate current distance based on the object's bounding box width
            box, _ = objectInfo[0]
            device_config['current_distance'] = str(max(0, 100 - box[2]))
            ser.write(b"FOLLOW\n")
        else:
            device_config['current_distance'] = "N/A"
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