import cv2 as cv

camera = cv.VideoCapture(0)
width = 640
height = 480
camera.set (cv.CAP_PROP_FRAME_WIDTH, width) 
camera.set(cv.CAP_PROP_FRAME_HEIGHT, height)

while True:
    result, frame = camera.read()  # Read frame from the camera
    if not result:
        break
    
    cv.imshow("USB Camera Test", frame)

    if cv.waitKey(1) & 0xFF == ord("q"):
        break

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