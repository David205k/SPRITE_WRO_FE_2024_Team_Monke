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
sys.path.append('/home/monke/WRO FE 2024 (Repository)/src/programs/modules')
sys.path.append('/home/monke/WRO FE 2024 (Repository)/src/programs')
from monke_hat import Car
from component_params import *
import cv2
from RPi import GPIO
import pygame
import time
import csv
from Traffic_sign.Traffic_sign import Traffic_sign
from parameters import * 

import threading

MAX_SPEED = 20 # maximum motor speed
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

    # print green bbox and coordinates
    if green_sign.x != -999:
        car.frame = green_sign.draw_bbox(car.frame, (0,255,0))
        text_pos = (50,camera["shape"][1] - 50)
        car.frame = cv2.putText(car.frame, f"({green_sign.map_x:.2f}, {green_sign.map_y:.2f})",
                                text_pos, font, 1, (0,255,0), 1, line)

    # print red bbox and coordinates
    if red_sign.x != -999:
        car.frame = red_sign.draw_bbox(car.frame, (0,0,255))
        text_pos = (camera["shape"][0] - 200,camera["shape"][1] - 50)
        car.frame = cv2.putText(car.frame, f"({red_sign.map_x:.2f},{red_sign.map_y:.2f})",
                                text_pos, font, 1, (0,0,255), 1, line)
        
    prev_time = cur_time
    cur_time = time.time()
    fps = 1 / (cur_time-prev_time)
    car.frame = cv2.putText(car.frame, f"fps: {fps:.2f}",
                    (30,30), font, 1, (255,0,0), 1, line)
    cv2.imshow("Camera", car.frame)


def background_tasks():

    try:
        while True:
            car.read_sensors()
            car.read_button()
            car.compass.set_home(car.read_button())

            car.get_frame()

            green_sign.detect_sign(frame=car.frame, min_pixel_h=20, min_pixel_w=20)
            red_sign.detect_sign(frame=car.frame, min_pixel_h=20, min_pixel_w=20)

            show_visuals()

            out.write(car.frame)
            if cv2.waitKey(1) & 0xFF == ord('q'): #break out of loop if 'q' is pressed
                out.release()
                cv2.destroyAllWindows()
                break

    except Exception as e:
        print(f"Error in background: {e}")

def map(var, min1, max1, min2, max2):
    return ((var - min1) / (max1-min1)) * (max2 - min2) + min2

prev_button_x = False
prev_button_o = False
prev_button_tri = False

button_x = False
button_o = False
button_tri = False

up_down_prev = 0

counter = 0

try:
    car.start_cam()
    # background_thread = threading.Thread(target=background_tasks, daemon=True)
    # background_thread.start()
    start = False
    while True:

        if car.read_button():
            start = True

        if start: 
            # Get events from the controller
            pygame.event.pump()
            car.read_sensors()
            
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

            if button_x == 1 and prev_button_x == 0:
                counter+=1
                if counter % 2 == 1:
                    start_t = time.time()
                    print("Started timing...")
                else:
                    end_t = time.time()
                    print("Ended timing...")
                    dist = 100
                    print("speed: ", dist/(end_t-start_t), " cm/s")
            if button_o == 1 and prev_button_o == 0:
                pass
                # car.compass.set_home()
                # car.LED.rgb(255,0,0)
                # time.sleep(0.3)

            if button_tri == 1:
                break
            # car.print_sensor_vals()

            car.get_frame()

            green_sign.detect_sign(frame=car.frame, min_pixel_h=20, min_pixel_w=20)
            red_sign.detect_sign(frame=car.frame, min_pixel_h=20, min_pixel_w=20)

            show_visuals()

            out.write(car.frame)
            if cv2.waitKey(1) & 0xFF == ord('q'): #break out of loop if 'q' is pressed
                out.release()
                cv2.destroyAllWindows()
                break

except Exception as E:
    print(f"{E} occured. Exiting...")

finally:
    # background_thread.join()
    out.release()
    car.LED.off()
    joystick.quit()
    pygame.quit()
    cv2.destroyAllWindows() 
    GPIO.cleanup()
