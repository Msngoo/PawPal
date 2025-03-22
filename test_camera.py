import cv2 as cv

camera = cv.VideoCapture(0)
width = 640
height = 480
camera.set (cv.CAP_PROP_FRAME_WIDTH, width) 
camera.set(cv.CAP_PROP_FRAME_HEIGHT, height)

while True:
    ret, frame = camera.read()  # Read frame from the camera
    if not ret:
        break
    
    cv2.imshow("USB Camera Test", frame)

camera.release()
cv.destroyAllWindows()


#Version2

'''
cap = cv.VideoCapture(0)
width = 640
height = 480
cap.set(cv.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, height)

while (1):
       ret, frame = cap.read()
       cv.imshow("USB Cam Vid", frame)

cv.destroyAllWindows()
exit()'
'''