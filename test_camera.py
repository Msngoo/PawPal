#!/usr/bin/env python3
import cv2
import glob

def main():
    # Find all video devices in /dev/
    device_paths = glob.glob('/dev/video*')
    if not device_paths:
        print("No video devices found in /dev/video*. Please check the USB camera connection.")
        return

    cap = None
    for device in device_paths:
        print(f"Trying camera device: {device}")
        temp_cap = cv2.VideoCapture(device)
        if not temp_cap.isOpened():
            print(f"Failed to open {device}.")
            continue

        # Try to grab a frame to verify the device works
        ret, frame = temp_cap.read()
        if ret and frame is not None:
            print(f"Camera device {device} appears to be working.")
            cap = temp_cap
            break
        else:
            print(f"Device {device} opened but no frame could be captured.")
            temp_cap.release()

    if cap is None:
        print("No accessible camera was found. Please check your USB camera connection and driver.")
        return

    # Set a desired resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            print("Failed to capture frame from the camera. It might have been disconnected or is in use.")
            break

        cv2.imshow("Camera Test", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()