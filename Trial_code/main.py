import cv2
from PIL import Image
import time
from picamera2 import Picamera2

import RPi.GPIO as GPIO
import tb6612fng
import pwmControl
import ultrasonicSensor_control as us
import movement  # Import the movement functions

# Disable GPIO warnings and set up GPIO mode
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

# Initialize Picamera2
picam2 = Picamera2()
picam2.preview_configuration.main.size = (1920, 1000)  # Set preview size
picam2.preview_configuration.main.format = 'RGB888'  # Set preview format
picam2.start()  # Start the camera

# Initialize the DC motor
dcMotor = tb6612fng.motor(11, 13, 15)

# Initialize ultrasonic sensors
us_front = us.ultrasonic(16, 18)
us_left = us.ultrasonic(20, 21)
us_right = us.ultrasonic(22, 23)

# Initialize lap count and direction flag
lap_count = 1
reverse_direction = False  # Flag for direction reversal

# Define the traffic sign detection class
class trafficSign:
    minWidth = 30
    minHeight = 30

    X = 0
    Y = 0

    def __init__(self, bgr, lowerLimit, upperLimit, minDist):
        self.bgr = bgr
        self.height, self.width = 0, 0
        self.minDist = minDist
        mask = cv2.inRange(hsvImage, lowerLimit, upperLimit)  # Create a mask for color detection
        mask_ = Image.fromarray(mask)  # Convert mask to image
        self.bbox = mask_.getbbox()  # Get bounding box of the detected color

    def printBbox(self):
        if self.bbox is not None:
            x1, y1, x2, y2 = self.bbox
            self.height, self.width = abs(y2 - y1), abs(x2 - x1)
            if self.height > trafficSign.minHeight and self.width > trafficSign.minWidth:
                self.X, self.Y = (x1 + x2) / 2, (y1 + y2) / 2
                cv2.rectangle(frame, (x1, y1), (x2, y2), self.bgr, 5)
                self.dist = int((frame.shape[0] / self.height) * self.minDist)

# Main loop
while True:
    dcMotor.setSpeed(40)  # Set motor speed
    capture = picam2.capture_array()  # Capture image from camera
    global frame
    global hsvImage
    frame = capture
    frame = cv2.flip(frame, 0)  # Flip the frame vertically
    frame = cv2.flip(frame, 1)  # Flip the frame horizontally
    hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # Convert frame to HSV color space

    green = trafficSign((0, 255, 0), (40, 100, 100), (70, 255, 255), 8)  # Detect green color
    red = trafficSign((0, 0, 255), (0, 200, 130), (5, 255, 255), 8)  # Detect red color

    green.printBbox()  # Print bounding box for green color
    red.printBbox()  # Print bounding box for red color

    cv2.line(frame, (frame.shape[1] // 2, 0), (frame.shape[1] // 2, frame.shape[0]), (0, 255, 255), thickness=3)  # Draw center line
    cv2.putText(frame, "Dist: " + str(green.dist), (frame.shape[1] - 120, 440), cv2.FONT_HERSHEY_TRIPLEX, 1.0, (0, 255, 0), 1)  # Display green distance
    cv2.putText(frame, "Dist: " + str(red.dist), (20, 440), cv2.FONT_HERSHEY_TRIPLEX, 1.0, (0, 0, 255), 1)  # Display red distance

    cv2.imshow('Camera', frame)  # Show the frame

    distance_front = us_front.getDist()  # Get front distance from ultrasonic sensor
    distance_left = us_left.getDist()  # Get left distance from ultrasonic sensor
    distance_right = us_right.getDist()  # Get right distance from ultrasonic sensor

    if distance_front < 20:  # Example distance threshold to detect pillar
        if lap_count < 3:
            if green.dist > 0 and (frame.shape[1] // 2 < green.X < frame.shape[1]):
                movement.handle_green()  # Handle green pillar
            elif red.dist > 0 and (0 < red.X < frame.shape[1] // 2):
                movement.handle_red()  # Handle red pillar
        else:
            if reverse_direction:
                if green.dist > 0 and (frame.shape[1] // 2 < green.X < frame.shape[1]):
                    movement.handle_green_third_lap()  # Handle green pillar for third lap in reversed direction
                elif red.dist > 0 and (0 < red.X < frame.shape[1] // 2):
                    movement.handle_red_third_lap()  # Handle red pillar for third lap in reversed direction
            else:
                if green.dist > 0 and (frame.shape[1] // 2 < green.X < frame.shape[1]):
                    movement.handle_green()  # Handle green pillar for third lap in same direction
                elif red.dist > 0 and (0 < red.X < frame.shape[1] // 2):
                    movement.handle_red()  # Handle red pillar for third lap in same direction

    if distance_front < 10 and (distance_left < 10 or distance_right < 10):  # Obstacle avoidance logic
        if distance_left < distance_right:
            movement.turn_left()  # Turn left to avoid obstacle
        else:
            movement.turn_right()  # Turn right to avoid obstacle

    if cv2.waitKey(1) & 0xFF == ord('q'):  # Quit if 'q' is pressed
        GPIO.cleanup()  # Clean up GPIO
        break

    # Update lap count logic here based on specific field conditions
    # Example placeholder:
    if condition_for_end_of_lap():  # Placeholder for actual condition to check end of lap
        lap_count += 1
        if lap_count == 3:
            if red.dist > 0 and (0 < red.X < frame.shape[1] // 2):
                reverse_direction = True  # Reverse direction if last pillar is red
            else:
                reverse_direction = False  # Continue in the same direction if last pillar is green
        elif lap_count > 3:
            lap_count = 3  # Ensure it doesn't go beyond 3 laps

cv2.destroyAllWindows()  # Close all OpenCV windows
