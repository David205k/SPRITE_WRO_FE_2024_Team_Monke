from modules.monke_hat.Car import Car
from robot_config import *
from parameters_open import *
from helper_functions import *

from RPi import GPIO
from math import *
import time
import threading
import numpy as np
import cv2

# objects
car = Car(
    camera=camera,
    servo=servo,
    us_front=us_front,
    us_left=us_left,
    us_right=us_spare2,
    us_spare1=us_spare1,
    us_spare2=us_right,
    rgb=rgb,
    pb=pb,
    mDrvr=mDrvr
)

def reset_driving():
    #reset servo angle and motor speed to default
    car.servo.write(0)
    car.motor.speed(SPEED)

def drive_dist(dist, speed=SPEED):
    car.motor.speed(speed)
    car.servo.write(car.pid_straight((1,0,0)))
    time.sleep(dist/(abs(speed)/100 * MAX_SPEED_CMS))

    reset_driving()

def arc(radius:float, heading:float, speed:float=SPEED, tol:float = None, lower_tol:float = 0,  upper_tol:float = 0):

    car.heading = heading

    if tol != None:
        upper = car.heading+tol
        lower = car.heading-tol
    else:
        upper = car.heading+upper_tol
        lower = car.heading+lower_tol

    while not is_ang_in_range(car.compass_direction, lower, upper):
        car.turn(radius)
        car.motor.speed(speed)

    reset_driving()

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
    print(f"Curve to point, theta: {theta}, tan dist: {tan_dist}")

    tol = 5
    # first arc
    arc(radius,heading=confine_ang(car.heading+theta),tol=tol)

    # tangent
    drive_dist(tan_dist, SPEED)

    # returning arc
    arc(-radius,heading=confine_ang(car.heading-theta),tol=tol)

    return theta, tan_dist

prev_time = 0
cur_time = 0
def show_visuals():

    global prev_time
    global cur_time

    font, line = cv2.FONT_HERSHEY_SIMPLEX, cv2.LINE_AA

    # get fps
    prev_time = cur_time
    cur_time = time.time()
    fps = 1 / (cur_time-prev_time)
    car.frame = cv2.putText(car.frame, f"fps: {fps:.2f}",
                    (30,30), font, 1, (255,0,0), 1, line)
    
    cv2.imshow("Camera", car.frame)

def setup():
    car.start_cam()

def background():

    try:
        while True:
            car.read_sensors(True)
            car.read_button()
            car.compass.set_home(car.read_button())

            # car.get_frame()

            # show_visuals()
            
            if cv2.waitKey(1) & 0xFF == ord('q'): #break out of loop if 'q' is pressed
                cv2.destroyAllWindows()
                break
    except Exception as e:
        print(f"Error in background tasks: {e}")

def main():

    start = False
    can_turn = True
    no_of_turns = 0
    start_pos = ()
    car.driving_direction = "CW"

    print("Boot complete. \nPress the button to run.")  

    while True:

        # end the round
        if (no_of_turns == TOTAL_TURNS 
        and (car.front_dist <= start_pos[0]+10)): # stop at start position
            start = False
            print("Run finished.")
            break

        if car.but_press:
            start = True
            
            car.reset()     # reset all variables

            start_pos = (car.front_dist, car.left_dist, car.right_dist) # store starting position

            print("Program started.")
            print("Running...")
            car.LED.rgb(100,100,100)
            time.sleep(0.2)

        if start:
            # Main code start here
            car.LED.rgb(0,200,0)

            if (car.front_dist <= 80 and can_turn and
                  ((car.left_dist >= 100 and car.driving_direction == "ACW") or
                   (car.right_dist >= 100 and car.driving_direction == "CW"))
                ):
                print(f"Turning. Front: {car.front_dist:.0f} Left: {car.left_dist:.0f} Right {car.left_dist:.0f}")

                no_of_turns += 1
                can_turn = False

                sign = 1 if car.driving_direction == "CW" else -1

                car.heading += 90*sign
                radius = -11*sign

                car.heading = confine_ang(car.heading)

                arc(radius,car.heading, upper_tol=10, lower_tol=-15)

            else:

                car.motor.speed(SPEED)
                car.servo.write(car.pid_straight((1,0,0)))

                if car.front_dist >= 130:
                    can_turn = True

        else:
            car.inactive()

if __name__=="__main__":

    try:
        setup()
        background_thread = threading.Thread(target=background, daemon=True)
        background_thread.start()
        try:
            main()
        except Exception as E:
            print(f"Error in main: {E}")
    except KeyboardInterrupt:
        pass
    finally: 
        # clean up any used resources (precaution)
        car.inactive()
        background_thread.join()
        GPIO.cleanup()
        cv2.destroyAllWindows()