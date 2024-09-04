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

# two pins are connected to the start button as safeguard
startBut1 = 16
startBut2 = 18
GPIO.setup(startBut1,GPIO.IN)
GPIO.setup(startBut2,GPIO.IN)

# camera vision 
# Define HSV color ranges for blue and orange
blue_lower, blue_upper = np.array([100, 20, 50]), np.array([150, 110, 90])
orange_lower, orange_upper = np.array([0, 145, 80]), np.array([10, 255, 150])

picam2 = Picamera2()
picam2.preview_configuration.main.size=(1920,1000)
picam2.preview_configuration.main.format = 'RGB888'
picam2.start()

def getRoundDirection(hsv_frame, frame):
    BlueAngleSum = 0
    OranAngleSum = 0
    BlueMean = 0
    OranMean = 0
    for d in range(0,5):
        # Create masks for blue and orange
        blue_mask = cv2.inRange(hsv_frame, blue_lower, blue_upper)
        orange_mask = cv2.inRange(hsv_frame, orange_lower, orange_upper)
        # Use Hough Line Transform to detect lines
        lineB = cv2.HoughLinesP(blue_mask, 1, np.pi / 180, threshold=100, minLineLength=100, maxLineGap=5)
        lineO = cv2.HoughLinesP(orange_mask, 1, np.pi / 180, threshold=100, minLineLength=100, maxLineGap=5)
        
        # Check if any lines were found
        if lineB is not None:
            for line in lineB:
                xB1, yB1, xB2, yB2 = line[0]
        
            # Calculate the angle of the line
            angleB = np.degrees(np.arctan2(yB2 - yB1, xB2 - xB1))
            BlueAngleSum += angleB
            print(f" Blue Line from ({xB1}, {yB1}) to ({xB2}, {yB2}) has an angle of {angleB:.2f} degrees")
    
            # Draw the line on the image
            cv2.line(frame, (xB1, yB1), (xB2, yB2), (0, 255, 0), 2)

        if lineO is not None:
            for line in lineO:
                xO1, yO1, xO2, yO2 = line[0]
        
            # Calculate the angle of the line
            angleO = np.degrees(np.arctan2(yO2 - yO1, xO2 - xO1))
            OranAngleSum += angleO
            print(f"Orange Line from ({xO1}, {yO1}) to ({xO2}, {yO2}) has an angle of {angleO:.2f} degrees")
    
            # Draw the line on the image
            cv2.line(frame, (xO1, yO1), (xO2, yO2), (0, 255, 0), 2)
    BlueMean = abs(BlueAngleSum)/5
    OranMean = abs(OranAngleSum)/5
    if BlueMean > OranMean:
        return "ACW"
    else:
        return "CW"



def main():
    frame = picam2.capture_array()
    frame = cv2.flip(frame, 0) # Flip vertically
    frame = cv2.flip(frame, 1) # Flip horizontally

    # Convert the frame to HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    blue_mask = cv2.inRange(hsv_frame, blue_lower, blue_upper)
    orange_mask = cv2.inRange(hsv_frame, orange_lower, orange_upper)

    while True:
        if True: #GPIO.input(startBut1) and GPIO.input(startBut2)
            start = True

            # reset variabls
            headingDirection = noOfTurns = 0
            canTurn = True

            drivingDirection = getRoundDirection(hsv_frame, frame)
            print(f"direction: {drivingDirection}, start: {start}")

        while True:
            cv2.imshow('preview', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'): #break out of loop if 'q' is pressed
                break

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        picam2.stop()
        cv2.destroyAllWindows() 
        GPIO.cleanup()