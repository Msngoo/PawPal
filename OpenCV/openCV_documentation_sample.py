import cv2
import serial
import time
import threading
import numpy as np
import pyaudio

# SERIAL COMMUNICATION SETUP:
# The Raspberry Pi communicates with the Arduino via the USB serial port.
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
ser.flush()

# OPENCV SETUP (unchanged):
classNames = []
classFile = "/home/eg1004/Desktop/Object_Detection_Files/coco.names"
with open(classFile, "rt") as f:
    classNames = f.read().rstrip("\n").split("\n")

configPath = "/home/eg1004/Desktop/Object_Detection_Files/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
weightsPath = "/home/eg1004/Desktop/Object_Detection_Files/frozen_inference_graph.pb"

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
                    cv2.rectangle(img, box, color=(0,255,0), thickness=2)
                    cv2.putText(img, classNames[classId - 1].upper(),
                                (box[0] + 10, box[1] + 30),
                                cv2.FONT_HERSHEY_COMPLEX, 1, (0,255,0), 2)
                    cv2.putText(img, str(round(confidence*100,2)),
                                (box[0] + 200, box[1] + 30),
                                cv2.FONT_HERSHEY_COMPLEX, 1, (0,255,0), 2)
    return img, objectInfo

# NEW FUNCTION FOR BARK DETECTION USING THE MICROPHONE ON CARD 1:
def detect_bark():
    CHUNK = 1024
    FORMAT = pyaudio.paInt16
    CHANNELS = 1             # Our I2S mic outputs a mono signal
    RATE = 44100             # Standard sampling rate for audio
    THRESHOLD = 1000         # Amplitude threshold; adjust via testing

    p = pyaudio.PyAudio()
    # With the device forced to card 1 via ALSA, we explicitly set the device index to 1.
    device_index = 1

    stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    frames_per_buffer=CHUNK,
                    input_device_index=device_index)
    
    while True:
        data = stream.read(CHUNK, exception_on_overflow=False)
        audio_data = np.frombuffer(data, dtype=np.int16)
        # If any sample exceeds the threshold, send "BARK" over serial
        if np.max(np.abs(audio_data)) > THRESHOLD:
            ser.write(b"BARK\n")
            time.sleep(1)  # Delay prevents rapid repeated triggers
    stream.stop_stream()
    stream.close()
    p.terminate()

if __name__ == "__main__":
    # Start the bark detection thread (mic is now fixed to card 1)
    bark_thread = threading.Thread(target=detect_bark, daemon=True)
    bark_thread.start()
    
    # Setup for OpenCV video capture
    cap = cv2.VideoCapture(0)
    cap.set(3, 640)
    cap.set(4, 480)
    
    # Main loop for video/dog detection and serial commands
    last_rotation_time = time.time()
    rotation_interval = 30  # seconds
    
    while True:
        success, img = cap.read()
        result, objectInfo = getObjects(img, 0.45, 0.2, objects=['dog'])
        
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