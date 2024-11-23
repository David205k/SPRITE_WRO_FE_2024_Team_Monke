# Shell command for testing camera: "libcamera-hello"

import cv2
from picamera2 import Picamera2

picam2 = Picamera2()
picam2.preview_configuration.main.size=(1920,1000)
picam2.preview_configuration.main.format = 'RGB888'
picam2.start()

while (True):

    im = picam2.capture_array()

    im = cv2.resize(im, (640,480))
    im = cv2.flip(im, 0) # Flip vertically
    im = cv2.flip(im, 1) # Flip horizontally
    
    cv2.imshow('preview', im)

    if cv2.waitKey(1) == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()