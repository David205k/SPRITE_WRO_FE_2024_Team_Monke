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
    Circle: Record Video
    X: Time robot to get robot speed
    Right Joystick: Motor Speed
    Left Joystick: Servo Angle
"""
import sys
sys.path.append('/home/monke/WRO FE 2024 (Repository)/src/programs')

from modules.monke_hat.Car import Car
from programs.robot_config import *
import cv2
from RPi import GPIO
import pygame
import time
from modules.Traffic_sign.Traffic_sign import Traffic_sign
from parameters import * 


# Initialize pygame and the joystick
pygame.init()
pygame.joystick.init()

# Assuming there's one controller connected
joystick = pygame.joystick.Joystick(0)
joystick.init()

print("Controller name:", joystick.get_name())

car = Car(
    camera=camera,
    servo=servo,
    us_front=us1,
    us_left=us2,
    us_right=us5,
    us_spare1=us4,
    us_spare2=us3,
    rgb=rgb,
    pb=pb,
    mDrvr=mDrvr 
)

MAX_SPEED = 50 # maximum motor speed
MAX_ANGLE = car.servo.maxAng # maximum motor angle

green_sign = Traffic_sign(GREEN_SIGN)
red_sign = Traffic_sign(RED_SIGN)

fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec for mp4 format
out = cv2.VideoWriter('output.mp4', fourcc, 20.0, (640, 480))

prev_time = 0
cur_time = 0
def show_visuals():

    global prev_time
    global cur_time

    font = cv2.FONT_HERSHEY_SIMPLEX
    line = cv2.LINE_AA

    car.frame = green_sign.draw_bbox(car.frame)
    car.frame = red_sign.draw_bbox(car.frame)
        
    prev_time = cur_time
    cur_time = time.time()
    fps = 1 / (cur_time-prev_time)
    car.frame = cv2.putText(car.frame, f"fps: {fps:.2f}",
                    (30,30), font, 1, (255,0,0), 1, line)
    cv2.imshow("Camera", car.frame)

def map(var, min1, max1, min2, max2):
    return ((var - min1) / (max1-min1)) * (max2 - min2) + min2

prev_button_x = False
prev_button_o = False
prev_button_tri = False
up_down_prev = 0

button_x = False
button_o = False
button_tri = False

record = False

time_robot = False

try:
    car.start_cam()
    start = False
    while True:

        if car.read_button():
            start = True

        if start: 
            # Get events from the controller
            pygame.event.pump()
            # car.read_sensors()
            
            car.compass.set_home(car.read_button())

            # Access joystick and button values
            left_vertical = joystick.get_axis(1)  # Left stick vertical
            right_horizontal = joystick.get_axis(3)  # Right stick vertical

            prev_button_x = button_x
            prev_button_o = button_o
            button_x = joystick.get_button(0)  # X button
            button_o = joystick.get_button(1)  # o button
            button_tri = joystick.get_button(2) # triangle button

            dpad = joystick.get_hat(0) # dpad (list, shape is [4])

            speed = -round(map(left_vertical, -1, 1 , -MAX_SPEED, MAX_SPEED))
            servoAng = -round(map(right_horizontal, -1, 1, -MAX_ANGLE, MAX_ANGLE))

            car.motor.speed(speed)
            car.servo.write(servoAng)
            car.LED.rgb(0,255,0)

            # calculate robot speed
            if button_x == 1 and prev_button_x == 0:
                time_robot = not time_robot
                if time_robot:
                    start_t = time.time()
                    print("Started timing...")
                else:
                    end_t = time.time()
                    print("Ended timing...")
                    dist = 100 # cm
                    print("speed: ", dist/(end_t-start_t), " cm/s")

            # record video feed
            if button_o == 1 and prev_button_o == 0:
                record = not record
                if record: print("Started recording...")
                else: print("Ended recording")

            # break out of loop
            if button_tri == 1:
                break

            if record:
                out.write(car.frame)

            # car.get_frame()
            # green_sign.detect_sign(frame=car.frame)
            # red_sign.detect_sign(frame=car.frame)
            # show_visuals()


            if cv2.waitKey(1) & 0xFF == ord('q'): #break out of loop if 'q' is pressed
                out.release()
                cv2.destroyAllWindows()
                break

except Exception as E:
    print(f"{E} occured. Exiting...")

finally:
    out.release()
    car.LED.off()
    joystick.quit()
    pygame.quit()
    cv2.destroyAllWindows() 
    GPIO.cleanup()
