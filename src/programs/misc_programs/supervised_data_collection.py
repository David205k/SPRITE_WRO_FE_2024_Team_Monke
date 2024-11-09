"""
Run this program to collect driving data while controlling the robot remotely 
using a ps4 controller. 

Steps:
1. Connect PS4 controller to raspberry pi. (if not already previously paired)
    > Open RPi in monitor/vnc
    > Open bluetooth >> "Add Device"
    > Press and hold the "Share" and "PS" button on the PS4 controller 
    until the LED on the controller starts flashing  
    > Select controller
2. Run this program
3. Press X to start recording data.
4. Drive a few rounds.
5. Press X to stop recording.
6. Press triangle to quit. >> Values will be saved to csv file.
7. Import csv file to your laptop and run training to create joblib file.
8. Run supervised_test.py using the joblib file generated to test the supervise training.

Modify MAX_SPEED and MAX_ANGLE to control max motor speed and servo angle
when using joystick.

Controls:
    Triangle: Quit
    Circle: Set compass home
    X: Start/Stop reccording
    Dpad Up/Down: Set Dist
    Dpas Left/Right: Set CW/ACW
    Right Joystick: Motor Speed
    Left Joystick: Servo Angle

Record:
    Program will store ultrasonic sensor, compass, dist and driving direction values into a csv file
    along with the corresponding action(motor speed and servo angle) made.

    Dist: How far from the center (1-3, 1: Close, 2: Middle, 3: Far) 
    Driving_direction: Clockwise or anticlockwise
"""

import sys
sys.path.append('/home/monke/WRO FE 2024 (Repository)/src/programs')

from monke_hat import Car
from component_params import *
import cv2
from RPi import GPIO
import pygame
import time
import csv

from picamera2 import Picamera2

MAX_SPEED = 100 # maximum motor speed
MAX_ANGLE = 45 # maximum motor angle

# Initialize pygame and the joystick
pygame.init()
pygame.joystick.init()

# Assuming there's one controller connected
joystick = pygame.joystick.Joystick(0)
joystick.init()

print("Controller name:", joystick.get_name())

car = Car.Car(
    camera=camera,
    wheelBase=wheelBase,
    servo=servo,
    us_front=us4,
    us_left=us2,
    us_right=us5,
    us_spare1=us1,
    us_spare2=us3,
    rgb=rgb,
    pb=pb,
    mDrvr=mDrvr 
)

front = []
left = []
right = []
compass = []
direction = []
dist = []
ang = []
m_speed = []

def map(var, min1, max1, min2, max2):

    return ((var - min1) / (max1-min1)) * (max2 - min2) + min2

# Function to save calibration data to CSV
def save_to_csv(front, left, right, compass, direction, dist, ang, m_speed, filename="training_data.csv"):
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        
        # Write headers
        writer.writerow(["Front", "Left", "Right", "Compass", "Direction", "Dist", "Angle", "Motor Speed"])
        
        # Write data
        print(zip(front, left, right, compass, direction, dist, ang, m_speed))
        for row in zip(front, left, right, compass, direction, dist, ang, m_speed):
            writer.writerow(row)
    
    print(f"Training data saved to {filename}")



record = False

prev_button_x = False
prev_button_o = False
prev_button_tri = False

button_x = False
button_o = False
button_tri = False
up_down_prev = 0

prev_time = 0
_dist = 1

driving_direction = 0 # 0 for cw, 1 for acw

try:
    car.start_cam()
    while True:

        frame = car.get_frame()
        cv2.imshow("Camera", frame)

        # Get events from the controller
        pygame.event.pump()
        car.read_sensors()
        
        car.compass.set_home(car.read_button())

        # Access joystick and button values
        left_vertical = joystick.get_axis(1)  # Left stick vertical
        right_horizontal = joystick.get_axis(3)  # Right stick vertical

        prev_button_x = button_x
        button_x = joystick.get_button(0)  # X button
        button_o = joystick.get_button(1)  # o button
        button_tri = joystick.get_button(2) # triangle button

        dpad = joystick.get_hat(0)

        speed = -round(map(left_vertical, -1, 1 , -50, 50))
        servoAng = -round(map(right_horizontal, -1, 1, -45, 45))

        car.motor.speed(speed)
        car.servo.write(servoAng)

        if button_x == 1 and prev_button_x == 0:
            record = not record
        
        if button_o == 1:
            car.compass.set_home()


        if dpad[0] == 1: # if right button on dpad pressed, driving direction is cw
            driving_direction=0
        elif dpad[0] == -1: # if right button on dpad pressed, driving direction is acw
            driving_direction=1 

        if up_down_prev != 1 and dpad[1] == 1:
            _dist += 1
            if _dist == 4:
                _dist = 1
        if up_down_prev != -1 and dpad[1] == -1:
            _dist -= 1
            if _dist == 0:
                _dist = 3

        if driving_direction==0:
            color = (0,0,100)  
        elif driving_direction==1:
            color = (100,0,100)

        if record:

            # print("Recording data")
            car.LED.rgb(100, 0, 0)

            if (time.time() - prev_time) >= 0.2:
                front.append(car.front_dist)
                left.append(car.left_dist)
                right.append(car.right_dist)
                compass.append(car.compass_direction)
                direction.append(driving_direction)
                dist.append(_dist)
                ang.append(servoAng)
                m_speed.append(speed)

                prev_time = time.time()

        else:
            r, g, b = color
            car.LED.rgb(r,g,b)


        if cv2.waitKey(1) & 0xFF == ord('q'): # triangle or q on keyboard
            cv2.destroyAllWindows()
            break 
        
        up_down_prev = dpad[1]

        car.print_sensor_vals()

        # print(f"Direction: {driving_direction}, Compass: {car.compass_direction}, Dist: {_dist}, Record: {record}")



except KeyboardInterrupt:
    print("Exiting...")

finally:
    save_to_csv(front, left, right, compass, direction, dist, ang, m_speed)
    car.LED.off()
    joystick.quit()
    pygame.quit()
    car.picam2.stop()
    cv2.destroyAllWindows()     
    GPIO.cleanup()
