import RPi.GPIO as GPIO # use RPi library for controlling GPIO pins
import math
import time
from gpiozero.pins.pigpio import PiGPIOFactory
from collections import deque

# modules for controlling components
import modules.Tb6612fngControl as Tb6612fng
import modules.RGBLEDControl as RGB
import modules.HMC5883LControl as HMC5883L
import modules.ServoControl_gpiozero as myservo
from gpiozero import DistanceSensor

import cv2
from picamera2 import Picamera2
import numpy as np

# camera vision 
# Define HSV color ranges for blue and orange
blue_lower, blue_upper = np.array([100, 20, 50]), np.array([150, 110, 90])
orange_lower, orange_upper = np.array([0, 145, 80]), np.array([10, 255, 150])

picam2 = Picamera2()
picam2.preview_configuration.main.size=(1920,1000)
picam2.preview_configuration.main.format = 'RGB888'
picam2.start()

def getRoundDirection(hsv_frame):

    # Create masks for blue and orange
    blue_mask = cv2.inRange(hsv_frame, blue_lower, blue_upper)
    orange_mask = cv2.inRange(hsv_frame, orange_lower, orange_upper)



def main():

    while True:
        frame = picam2.capture_array()
        frame = cv2.flip(frame, 0) # Flip vertically
        frame = cv2.flip(frame, 1) # Flip horizontally

        # Convert the frame to HSV color space
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        blue_mask = cv2.inRange(hsv_frame, blue_lower, blue_upper)
        orange_mask = cv2.inRange(hsv_frame, orange_lower, orange_upper)


        getRoundDirection(hsv_frame)

        # Use Hough Line Transform to detect lines
        lineB = cv2.HoughLinesP(blue_mask, 1, np.pi / 180, threshold=100, minLineLength=100, maxLineGap=5)
        lineO = cv2.HoughLinesP(orange_mask, 1, np.pi / 180, threshold=100, minLineLength=100, maxLineGap=5)
        
        # Check if any lines were found
        if lineB is not None:
            for line in lineB:
                x1, y1, x2, y2 = line[0]
        
            # Calculate the angle of the line
            angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
            print(f" Blue Line from ({x1}, {y1}) to ({x2}, {y2}) has an angle of {angle:.2f} degrees")
    
            # Draw the line on the image
            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        if lineO is not None:
            for line in lineO:
                x1, y1, x2, y2 = line[0]
        
            # Calculate the angle of the line
            angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
            print(f"Orange Line from ({x1}, {y1}) to ({x2}, {y2}) has an angle of {angle:.2f} degrees")
    
            # Draw the line on the image
            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        cv2.imshow('preview', frame)
        cv2.imshow('orange mask', orange_mask)

        if cv2.waitKey(1) & 0xFF == ord('q'): #break out of loop if 'q' is pressed
            break

    picam2.stop()
    cv2.destroyAllWindows() 
    GPIO.cleanup()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        picam2.stop()
        cv2.destroyAllWindows() 
        GPIO.cleanup()