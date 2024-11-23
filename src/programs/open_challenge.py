"""
This program is to be run for the WRO Future Engineers Open Challenge.
"""

from modules.monke_hat.Car import Car
from robot_config import *
import helper_functions

from math import *
from RPi import GPIO
import time
import numpy as np
import cv2

GPIO.setwarnings(False) # turn off warnings for pins

# initialise car object
car = Car(
    camera=camera,
    servo=servo,
    us_front=us_spare1,
    us_left=us_left,
    us_right=us_spare2,
    us_spare1=us_front,
    us_spare2=us_right,
    rgb=rgb,
    pb=pb,
    mDrvr=mDrvr
)

can_turn = True

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

    if lower_bound < upper_bound:
        return lower_bound <= ang <= upper_bound
    else:
        return (ang >= lower_bound) or (ang <= upper_bound)

# functions 
def get_angular_diff(intendedAngle: float, currentAng: float) -> float: # cw => -ve ccw => +ve 
    """
    Get the differenc between two angles

    Parameters
    ----------
    intendedAngle: float
        Difference from this position
    currentAng: float
        Position you are currently facing
    angDiff: float
        difference between the two positions| +ve: acw, -ve: cw
    """

    angDiff = intendedAngle - currentAng

    angDiff = (360 - angDiff) if angDiff > 180 else (360 + angDiff) if angDiff < -180 else angDiff
    return -angDiff

def decide_action(action_queue: deque):

    global can_turn
    if action_queue:
        pass
    elif car.front_dist <= 100 and car.right_dist >= 100 and can_turn: # right angle turn
        can_turn = False

        turn_radius = MIN_WALL_DIST + ROBOT_WIDTH/2
        if car.driving_direction == "CW":
            car.heading += 90 
            turn_radius = -turn_radius
            print(f"added heading: {car.heading}")
        elif car.driving_direction == "ACW":
            car.heading -= 90 
            turn_radius = turn_radius
        action_queue = deque([["turn", turn_radius]])

    else:
        action_queue = deque([["run"]])

    print("heading", car.heading)
    car.heading = confine_ang(car.heading)
    return action_queue


def excecute_action(action_queue: deque):
    """
    Perform action based on action queue.
    """

    completed = None

    if action_queue:
        match action_queue[0][0]:

            case "turn":
                speed = 60 + min(40,40*abs(((confine_ang(car.heading - car.compass_direction))/90)))
                print(speed, (confine_ang(car.heading - car.compass_direction)))
                car.motor.speed(speed)
                tol = 10   # angular tolerance in degrees

                turn_radius = action_queue[0][1] 
                car.turn(turn_radius)

                if is_ang_in_range(car.compass_direction, 
                                confine_ang(car.heading-15), 
                                confine_ang(car.heading+10)): 
                    completed = True
                # print(car.heading, confine_ang(car.heading-tol), confine_ang(car.heading+tol))

            case "run": 
                car.motor.speed(SPEED)
                car.servo.write(car.pid_straight((1,0,0),car.heading))
                completed = True
        
    if completed:
        action_queue.popleft()

    return action_queue

def main():
    """
    Main program. Add your code here.
    """
    global can_turn
    # variables
    action_queue = deque()
    start = False 

    print("Boot complete. \nPress the button to run.")
    # main program loop
    while True:

        car.read_sensors(True)
        # print(f"heading {car.heading}")
        ang_diff = car.compass_direction - car.heading

        # print(f"act left: {act_left} left: {car.left_dist} act right: {act_right} right: {car.right_dist}")
        car.compass.set_home(car.read_button())

        if car.read_button():
            start = True
            print("Program started.")
            print("Running...")
            car.LED.rgb(100,100,100)
            time.sleep(0.2)

        if start: # Perform open challenge
            car.LED.rgb(0,200,0)
            # e = ((30 - act_left) / 35) * 45
            # p = 1 * e
            # print(p)
            # car.servo.write(-p)
            # car.motor.speed(20)

            action_queue = decide_action(action_queue)
            # print(action_queue)
            action_queue = excecute_action(action_queue) 

            if car.front_dist >= 110:
                can_turn = True
        else: 
            car.inactive()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally: 
        # clean up any used resources (precaution)
        GPIO.cleanup()
        cv2.destroyAllWindows()