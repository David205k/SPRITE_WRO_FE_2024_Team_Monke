import cv2
from picamera2 import Picamera2
import numpy as np

# Define HSV color ranges for blue and orange
blue_lower = np.array([80, 130, 50])
blue_upper = np.array([140, 180, 90])

orange_lower = np.array([0, 190, 80])
orange_upper = np.array([20, 255, 150])

picam2 = Picamera2()
picam2.preview_configuration.main.size=(1920,1000)
picam2.preview_configuration.main.format = 'RGB888'
picam2.start()

while (True):

    im = picam2.capture_array()
    im = cv2.flip(im, 0) # Flip vertically
    im = cv2.flip(im, 1) # Flip horizontally


    # Convert the frame to HSV color space
    hsv_frame = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

    # Create masks for blue and orange
    blue_mask = cv2.inRange(hsv_frame, blue_lower, blue_upper)
    orange_mask = cv2.inRange(hsv_frame, orange_lower, orange_upper)

    # Scan the frame from top to bottom
    for y in range(hsv_frame.shape[0]):  # Loop over rows (height)
        # Get the row masks
        blue_row = blue_mask[y, :]
        orange_row = orange_mask[y, :]

        # Check if blue or orange is detected first
        if np.any(blue_row):  # Check if any blue pixel is present
            print("Blue detected first at row", y)
            break
        elif np.any(orange_row):  # Check if any orange pixel is present
            print("Orange detected first at row", y)
            break
    else:
        print("No blue or orange detected in the frame")

    # print(f"colour: {hsv_frame[44, 40]}")

    cv2.imshow('preview', im)

    if cv2.waitKey(1) == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()