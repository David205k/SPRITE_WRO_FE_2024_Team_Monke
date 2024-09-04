import cv2
import numpy as np

# Define HSV color ranges for blue and orange
blue_lower = np.array([0, 150, 50])
blue_upper = np.array([140, 255, 255])

orange_lower = np.array([10, 100, 100])
orange_upper = np.array([25, 255, 255])

# Capture a frame from the camera
cap = cv2.VideoCapture(0)  # Replace 0 with the appropriate camera index if needed


while True:
    ret, frame = cap.read()
    if ret:
        # Convert the frame to HSV color space
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

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

    # Display the resulting frame
    cv2.imshow('Laptop Camera', frame)

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera
cap.release()
cv2.destroyAllWindows()