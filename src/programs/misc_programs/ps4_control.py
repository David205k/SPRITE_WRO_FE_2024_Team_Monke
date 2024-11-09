"""
Run this program to control the robot remotely using the ps4 controller. 

Steps:
1. Connect PS4 controller to raspberry pi. (if not already previously paired)
    > Open RPi in monitor/vnc
    > Open bluetooth >> "Add Device"
    > Press and hold the "Share" and "PS" button on the PS4 controller 
    until the LED on the controller starts flashing  
    > Select controller
2. Run this program

Modify MAX_SPEED and MAX_ANGLE to control max motor speed and servo angle
when using joystick.

Controls:
    Triangle: Quit
    Circle: Set compass home
    Right Joystick: Motor Speed
    Left Joystick: Servo Angle
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

def map(var, min1, max1, min2, max2):
    return ((var - min1) / (max1-min1)) * (max2 - min2) + min2

prev_button_x = False
prev_button_o = False
prev_button_tri = False

button_x = False
button_o = False
button_tri = False

up_down_prev = 0

try:
    while True:
        # Get events from the controller
        pygame.event.pump()
        car.read_sensors()
        
        car.compass.set_home(car.read_button())

        # Access joystick and button values
        left_vertical = joystick.get_axis(1)  # Left stick vertical
        right_horizontal = joystick.get_axis(3)  # Right stick vertical

        button_x = joystick.get_button(0)  # X button
        button_o = joystick.get_button(1)  # o button
        button_tri = joystick.get_button(2) # triangle button

        dpad = joystick.get_hat(0) # dpad (list, shape is [4])

        speed = -round(map(left_vertical, -1, 1 , -MAX_SPEED, MAX_SPEED))
        servoAng = -round(map(right_horizontal, -1, 1, -MAX_ANGLE, MAX_ANGLE))

        car.motor.speed(speed)
        car.servo.write(servoAng)
        car.LED.rgb(0,255,0)

        if button_o == 1:
            car.compass.set_home()
            car.LED.rgb(255,0,0)
            time.sleep(0.3)
        if button_tri == 1:
            break

        car.print_sensor_vals()

except KeyboardInterrupt:
    print("Exiting...")

finally:
    car.LED.off()
    joystick.quit()
    pygame.quit()
    cv2.destroyAllWindows() 
    GPIO.cleanup()
