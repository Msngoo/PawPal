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

# BARK_THRESHOLD for 32-bit samples.
# In 32-bit mode, sample values can be much larger (range ~ -2^31 to 2^31-1).
# You may need to adjust this threshold based on your testing.
BARK_THRESHOLD = 100000

# NEW FUNCTION FOR BARK DETECTION USING THE I2S MICROPHONE ON CARD 1 (32-bit):
def detect_bark():
    CHUNK = 1024
    FORMAT = pyaudio.paInt32    # 32-bit audio input format
    CHANNELS = 1                # Assuming the I2S mic outputs mono audio
    RATE = 48000                # Sample rate (48 kHz)
    threshold = BARK_THRESHOLD  # Sensitivity threshold for 32-bit mode

    p = pyaudio.PyAudio()
    device_index = 1  # Using the fixed ALSA card 1

    try:
        stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE,
                        input=True, frames_per_buffer=CHUNK,
                        input_device_index=device_index)
    except Exception as e:
        # If unable to open the stream, terminate PyAudio.
        print("Error opening audio stream:", e)
        p.terminate()
        return

    while True:
        try:
            data = stream.read(CHUNK, exception_on_overflow=False)
        except Exception as e:
            continue
        # Convert the byte data into a NumPy array using 32-bit integers.
        audio_data = np.frombuffer(data, dtype=np.int32)
        if np.max(np.abs(audio_data)) > threshold:
            # If a "bark" is detected, send the "BARK" command.
            ser.write(b"BARK\n")
            time.sleep(1)  # Delay to mitigate rapid re-triggering

    stream.stop_stream()
    stream.close()
    p.terminate()

if __name__ == "__main__":
    # Start the bark detection thread (using 32-bit samples).
    bark_thread = threading.Thread(target=detect_bark, daemon=True)
    bark_thread.start()

    # Setup OpenCV video capture (for dog detection)
    cap = cv2.VideoCapture(0)
    cap.set(3, 640)  # Width
    cap.set(4, 480)  # Height

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