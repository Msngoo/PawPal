import cv2
import serial
import time
import threading
import numpy as np
import pyaudio

# SERIAL COMMUNICATION SETUP:
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
ser.flush()

# OPENCV SETUP:
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
                    cv2.rectangle(img, box, color=(0, 255, 0), thickness=2)
                    cv2.putText(img, className.upper(), (box[0] + 10, box[1] + 30),
                                cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(img, str(round(confidence * 100, 2)), (box[0] + 200, box[1] + 30),
                                cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
    return img, objectInfo

# UPDATED BARK DETECTION CONFIGURATION
def detect_bark():
    CHUNK = 1024
    FORMAT = pyaudio.paInt16    # Using 16-bit audio format
    CHANNELS = 1                # Mono configuration for single I2S mic
    RATE = 48000                # Standard sample rate 48 kHz
    # Threshold Explanation:
    # 16-bit audio samples range from -32768 to 32767.
    # Typical RMS values for a dog bark may range from about 10000 to 20000.
    # Setting the threshold here to 15000 means that when the RMS of the audio exceeds 15000,
    # it is considered a bark.
    threshold = 15000           

    p = pyaudio.PyAudio()
    device_index = 3            # Using device index 3 (as identified by arecord -l)

    try:
        dev_info = p.get_device_info_by_index(device_index)
    except Exception:
        p.terminate()
        return

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

if __name__ == "__main__":
    # Start bark detection thread
    bark_thread = threading.Thread(target=detect_bark, daemon=True)
    bark_thread.start()

    # Setup video capture
    cap = cv2.VideoCapture(0)
    cap.set(3, 640)
    cap.set(4, 480)

    last_rotation_time = time.time()
    rotation_interval = 30  # seconds

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