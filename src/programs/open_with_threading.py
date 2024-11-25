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
car = Car()

def reset_driving():
    #reset servo angle and motor speed to default
    car.servo.write(0)
    car.motor.speed(SPEED)

def drive_dist(dist, speed=SPEED):
    # drive a specified distance for a certain speed
    car.motor.speed(speed)
    car.servo.write(car.pid_straight((1,0,0)))
    time.sleep(dist/(abs(speed)/20 * MAX_SPEED_CMS))

    reset_driving()

def arc(radius:float, heading:float, speed:float=SPEED, tol:float = None, lower_tol:float = 0,  upper_tol:float = 0):

    car.heading = heading

    if tol != None:
        upper = car.heading+tol
        lower = car.heading-tol
    else:
        if radius >= 0: # ACW
            upper = car.heading-upper_tol
            lower = car.heading-lower_tol
        else: # CW
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

    tol = 3
    # first arc
    arc(radius,heading=confine_ang(car.heading+theta),tol=tol)

    # tangent
    drive_dist(tan_dist, SPEED)

    # returning arc
    arc(-radius,heading=confine_ang(car.heading-theta),tol=tol)

    return theta, tan_dist

def show_visuals():
    cv2.imshow("Camera", car.frame)

def setup():
    car.start_cam()

def background():

    try:
        while True:
            car.read_sensors()
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
    turn_time = 0

    print("Boot complete. \nPress the button to run.")  

    while True:

        # end the round
        if (no_of_turns == TOTAL_TURNS 
        and (car.front_dist <= start_pos[0]+10)): # stop at start position
            start = False
            print("Run finished.")
            

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
            car.LED.rgb(255,0,255) # pink

            if (car.front_dist <= 80 and can_turn and
                  (car.left_dist>=100 or car.right_dist>=100)
                ):
                car.LED.rgb(0,50,0) # white
                print(f"Turning. Front: {car.front_dist:.0f} Left: {car.left_dist:.0f} Right {car.left_dist:.0f}")

                no_of_turns += 1
                can_turn = False
                print(f"no of turns: {no_of_turns}")
                if no_of_turns == 1:
                    car.driving_direction = "ACW" if car.left_dist >= car.right_dist else "CW"
                    print(f"Driving_direction: {car.driving_direction}")

                sign = 1 if car.driving_direction == "CW" else -1

                car.heading += 90*sign
                radius = -20*sign

                arc(radius, car.heading, SPEED, lower_tol=5, upper_tol=5)
            else:

                car.motor.speed(SPEED)
                car.servo.write(car.pid_straight((2.5,0,0)))
                
                #if (car.left_dist<=15):
                #    curve_to_point(-30, 10, 20)
                #if (car.right_dist<=15):
                #    curve_to_point(30,-10, 10)
                #curve_to_point(5,5,5)
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