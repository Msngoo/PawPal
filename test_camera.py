#------------------------------------------------------------------------------------------------
#SECTION 1: Importing Libraries and Setting Up Pins
import cv2
import numpy as np

camera = cv2.VideoCapture(0)
width = 640
height = 480
camera.set (cv2.CAP_PROP_FRAME_WIDTH, width) 
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

while True:
    ret, frame = camera.read()  # Read frame from the camera
    if not ret:
        break
    
    cv2.imshow("USB Camera Test", frame)

camera.release()
cv2.destroyAllWindows()