from modules.monke_hat.Car import Car
from programs.robot_config import *
from parameters import *
from modules.Traffic_sign.Traffic_sign import Traffic_sign

from RPi import GPIO
import threading
from picamera2 import Picamera2
from collections import deque
import numpy as np
import cv2
from math import *
import time

# objects
car = Car(
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

# global vars
SPEED = 20
MIN_WALL_DIST = 25 # cm

exit_key_pressed = False

def confine_ang(ang):
    """
    Confine self.heading to 0 and 360 degrees
    """
    ang %= 360
    if ang < 0:
        ang += 360
    return ang

def is_ang_in_range(ang: float, lower_bound: float, upper_bound: float) -> bool:
    """
    Check if an angle is within a range.

    E.g. if ang=27, is_ang_in_range(ang, 0, 90) -> True
        if ang=27, is_ang_in_range(ang, 270, 90) -> True
        if ang-27, is_ang_in_range(ang, 90, 270) -> False
    Parameters
    ----------
    ang: float
        angle to check (from 0-360)
    lower_bound: float
        smaller value (from 0-360)
    upper_bound: float
        larger value (from 0-360)
    """ 

    lower_bound = confine_ang(lower_bound)
    upper_bound = confine_ang(upper_bound)

    if lower_bound < upper_bound:
        return lower_bound <= ang <= upper_bound
    else:
        return (ang >= lower_bound) or (ang <= upper_bound)

def curve_to_point(radius:float, x:float, y:float):
    """
    Drive to a point x distance to the side, and y distance infront of the robot.

    Car will curve on an arc, drive on the tangent, and curve back to face the 
    same direction.

    Parameters
    ----------
    radius: float
        Radius of arc in cm.
    x: float
        Lateral distance of desired position. +ve => right side | -ve =>left side
    y: float
        Distance of desired position to robot. 
    """
    theta, tan_dist = calculate_route(radius,x,y)

    tol = 5
    # first arc
    car.heading = confine_ang(car.heading+theta)
    while not is_ang_in_range(car.compass_direction, car.heading-tol, car.heading+tol):  
        car.turn(radius)
        car.motor.speed(SPEED)

    # tangent
    car.servo.write(0)
    car.motor.speed(SPEED)
    travelling_time = tan_dist / (SPEED/100 * MAX_SPEED_CMS)

    time.sleep(travelling_time)

    # returning arc
    car.heading = confine_ang(car.heading-theta)
    while not is_ang_in_range(car.compass_direction, car.heading-tol, car.heading+tol): 
        car.turn(-radius)
        car.motor.speed(SPEED)

def calculate_route(r:float, x:float, y:float):
    """
    Calculates the arc angle and length of tangent.

    Parameters
    ----------
    theta:  float
        Angle to turn to relative to the normal. -ve => acw, +ve cw
    tangent:   float
        Distance to travel on the tangent to the arcs. Cm
    """

    c2c_len = sqrt((abs(x)-2*r)**2+y**2)                             # length of line btw the 2 arc centers
    print("c2c_len", c2c_len)
    if y < r*2: raise ValueError("End point too close")
    if y < 0: raise ValueError("Y should  be infront of car")

    alpha = degrees(acos(r/(c2c_len/2)))                        # angle between radius and c2c line
    phi = degrees(atan2(y,abs(x)-2*r))                               # angle from c2c line to +ve x axis
    theta = 180 - alpha - phi                                   # angle of tangent to normal
    tangent = (y - 2*r*sin(radians(theta))) / cos(radians(theta))  # length of tangent
    if x < 0:
        return -theta, tangent
    else:
        return theta, tangent

def setup():
    car.start_cam()

def background():

    global exit_key_presesd

    prev_time = 0
    cur_time = 0

    while True:
        car.read_sensors()
        car.read_button()
        car.compass.set_home(car.read_button())

        car.get_frame()

        font = cv2.FONT_HERSHEY_SIMPLEX
        line = cv2.LINE_AA

        # detect green traffic sign
        if green_sign.detect_sign(frame=car.frame, min_pixel_h=20, min_pixel_w=20):
            # car.frame = green_sign.draw_bbox(car.frame, (0,255,0))
            # text_pos = (50,camera["shape"][1] - 50)
            # car.frame = cv2.putText(car.frame, f"({green_sign.map_x:.2f}, {green_sign.map_y:.2f})",
            #                         text_pos, font, 1, (0,0,255), 1, line)
            pass

        # detect red traffic sign
        if red_sign.detect_sign(frame=car.frame, min_pixel_h=20, min_pixel_w=20):
            # car.frame = red_sign.draw_bbox(car.frame, (0,0,255))
            # text_pos = (camera["shape"][0] - 200,camera["shape"][1] - 50)
            # car.frame = cv2.putText(car.frame, f"({red_sign.map_x:.2f},{red_sign.map_y:.2f})",
            #                         text_pos, font, 1, (0,0,255), 1, line)
            pass
        
        if cv2.waitKey(1) & 0xFF == ord('q'): #break out of loop if 'q' is pressed
            cv2.destroyAllWindows()
            exit_key_pressed = True
            break

def turn(heading_direction, radius, lower_tol, upper_tol):

    while not is_ang_in_range(car.compass_direction, heading_direction+lower_tol, heading_direction+upper_tol):
                    
        # decelerate as car reaches the direction
        speed = 60 + min(40,40*abs(((confine_ang(heading_direction - car.compass_direction))/90)))
        car.turn(radius)
        car.motor.speed(speed)

def main():

    global exit_key_pressed
    start = False
    can_turn = True
    car.driving_direction = "ACW"
    no_of_turns = 0

    start_pos = ()
    not_done = True

    print("Boot complete. \nPress the button to run.")  

    while True:

        if car.but_press:
            start = True
            # reset all variables
            car.reset()

            start_pos = (car.front_dist, car.left_dist, car.right_dist)

            print("Program started.")
            print("Running...")
            car.LED.rgb(100,100,100)
            time.sleep(0.2)
        
        if no_of_turns == 2 and start_pos[0]-10 <= car.front_dist <= start_pos[0]+10:
            start = False
            print("Round finished.")
            break


        if start:
            # Main code start here
            car.LED.rgb(0,200,0)

            # if (green_sign.map_x != -999 and green_sign.map_x < CAR_WIDTH/2
            # and green_sign.map_y < 60 and green_sign.map_y >= 44):

            #     x = green_sign.map_x - green_sign.width/2 - CAR_WIDTH / 2 - 10
            #     y = green_sign.map_y - 5
            #     r = 20

            #     curve_to_point(r, x, y)

            # elif (red_sign.map_x != -999 and red_sign.map_x > -CAR_WIDTH/2
            # and red_sign.map_y < 60 and red_sign.map_y >= 44):

            #     x = red_sign.map_x + red_sign.width/2 + CAR_WIDTH/2 + 10
            #     y = red_sign.map_y - 5
            #     r = 20

            #     theta, tan_dist = calculate_route(r,x,y)
            #     curve_to_point(r, theta, tan_dist)
            if no_of_turns == 1 and 150 - 3 <= car.front_dist <= 150 + 3 and not_done:
                not_done = False
                print("circling")
                if car.driving_direction == "ACW":

                    radius = 100 #(150 - car.right_dist - CAR_WIDTH/2 - 10)
                    start_ang = 270
                    end_ang = 90

                else: 
                    radius = -120#(150 - car.left_dist - CAR_WIDTH /2)
                    start_ang = 90
                    end_ang = 270

                print(radius)

                counter = 0
                can_count = True
                while not(counter == 3 and is_ang_in_range(car.compass_direction, end_ang-5,end_ang+5)):
                    if is_ang_in_range(car.compass_direction, start_ang-5, start_ang+5) and can_count:
                        counter += 1
                        can_count = False

                    if is_ang_in_range(car.compass_direction, end_ang-5, end_ang+5):
                        can_count = True
                    car.turn(radius)
                    car.motor.speed(80)
                car.heading = end_ang

            elif (car.front_dist <= 60 and can_turn and
                ((car.right_dist >= 100 and car.driving_direction == "CW") or 
                (car.left_dist >= 100 and car.driving_direction == "ACW"))):
                print("turning")

                no_of_turns += 1
                
                can_turn = False

                if no_of_turns  == 2:
                    turn_radius = start_pos[2] + CAR_WIDTH/2 if car.driving_direction == "CW" else start_pos[1] + CAR_WIDTH/2 - 5
                else:
                    turn_radius = 30#MIN_WALL_DIST + CAR_WIDTH/2

                lower_ang, upper_ang = -5, 5

                if car.driving_direction == "CW":
                    car.heading += 90 
                    turn_radius = -turn_radius
                elif car.driving_direction == "ACW":
                    car.heading -= 90 

                car.heading = confine_ang(car.heading)

                while not is_ang_in_range(car.compass_direction, car.heading+lower_ang, car.heading+upper_ang):
                                
                    # decelerate as car reaches the direction
                    speed = 20 + min(20,30*abs(((confine_ang(car.heading-car.compass_direction))/90)))
                    car.turn(turn_radius)
                    car.motor.speed(speed)

            # elif car.left_dist <= 10 or car.right_dist <= 10 and no_of_turns != 2:
            #     print("avoiding")
            #     x = 20 if car.left_dist <= 10 else -20
            #     curve_to_point(20, x, 41)

            else:

                car.motor.speed(SPEED)
                car.servo.write(car.pid_straight((1,0,0)))

                if car.front_dist >= 130:
                    can_turn = True

        else:
            car.inactive()

        # print(f"Front: {car.front_dist} Left: {car.left_dist} Right: {car.right_dist} Compass: {car.compass_direction} Heading: {car.heading}")

        if exit_key_pressed:
            break


if __name__=="__main__":

    try:
        setup()
        background_thread = threading.Thread(target=background, daemon=True)
        background_thread.start()
        main()
    except KeyboardInterrupt:
        pass
    finally: 
        # clean up any used resources (precaution)
        background_thread.join()
        GPIO.cleanup()
        cv2.destroyAllWindows()