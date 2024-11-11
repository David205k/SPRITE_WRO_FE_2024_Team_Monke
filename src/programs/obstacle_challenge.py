"""
This program is to be run for the WRO Future Engineers Open Challenge.
"""

from monke_hat import Car
from component_params import *
from parameters import *
from TrafficSign import Traffic_sign

from math import *
from RPi import GPIO
import time
from gpiozero.pins.pigpio import PiGPIOFactory
from collections import deque
from picamera2 import Picamera2
import numpy as np
import cv2

GPIO.setwarnings(False) # turn off warnings for pins
GPIO.setmode(GPIO.BOARD) # pin name convention used is pin numbers on board
factory = PiGPIOFactory()

# global variables
SPEED = 20
MIN_WALL_DIST = 30 # cm

can_turn = False

# initialise car object
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
    can_turn = True

    angDiff = (360 - angDiff) if angDiff > 180 else (360 + angDiff) if angDiff < -180 else angDiff
    return -angDiff

def curve_to_point(r, x, y):
    """
    Move laterally

    Curve on an arc, drive on the tangent, drive on the arc back to get straight again. Moved laterally overall. Arc are of 
    equal radius.

    Parameters
    ----------
    theta:  float
        Angle to turn to relative to the normal.
    hypo:   float
        Distance to travel on the tangent to the arcs.
    """

    c2c_len = sqrt((abs(x)-2*r)**2+y**2)                             # length of line btw the 2 arc centers
    print("c2c_len", c2c_len)
    if y < r*2: raise ValueError("End point too close")
    if y < 0: raise ValueError("Y should  be infront of car")

    alpha = degrees(acos(r/(c2c_len/2)))                        # angle between radius and c2c line
    print("alpha", alpha)
    phi = degrees(atan2(y,abs(x)-2*r))                               # angle from c2c line to +ve x axis
    theta = 180 - alpha - phi                                   # angle of tangent to normal
    print("phi", phi)
    hypo = (y - 2*r*sin(radians(theta))) / cos(radians(theta))  # length of tangent
    print("hypo", hypo)
    if x < 0:
        return -theta, hypo
    else:
        return theta, hypo

just_done_green = True
just_done_red = True
do_once = False
def decide_action(action_queue: deque):
    
    global can_turn
    global just_done_green
    global just_done_red
    if action_queue:
        pass
    elif green_sign.map_x != -999 and green_sign.map_x < CAR_WIDTH/2 and green_sign.map_y < 60 and green_sign.map_y >= 44 and not just_done_green:
        just_done_green = True
        x = green_sign.map_x - green_sign.width/2 - CAR_WIDTH / 2 - 10
        y = green_sign.map_y - 5
        r = 20
        theta, tan_dist = curve_to_point(r=r, x=x, y=y)

        required_time = tan_dist / (SPEED/100 * MAX_SPEED_CMS)

        heading =  confine_ang(car.heading+theta)

        print(f"theta:{theta}, required_time:{required_time}, tan_dist:{tan_dist} x:{x}, y:{y}")
        action_queue = deque([["turn avoid", r, heading], 
                                ["run till", required_time], 
                                ["turn avoid", -r, car.heading]])

    elif red_sign.map_x != -999 and red_sign.map_x > -CAR_WIDTH/2 and red_sign.map_y < 60 and red_sign.map_y >= 44 and not just_done_red:
        just_done_red = True
        x = red_sign.map_x + red_sign.width/2 + CAR_WIDTH / 2 + 10
        y = red_sign.map_y - 5
        r = 20
        theta, tan_dist = curve_to_point(r=r, x=x, y=y)

        required_time = tan_dist / (SPEED/100 * MAX_SPEED_CMS)

        heading =  confine_ang(car.heading+theta)

        print(f"theta:{theta}, required_time:{required_time}, tan_dist:{tan_dist} x:{x}, y:{y}")
        action_queue = deque([["turn avoid", -r, heading], 
                                ["run till", required_time], 
                                ["turn avoid", r, car.heading]])


    elif (car.front_dist <= 100 and can_turn and
        ((car.right_dist >= 100 and car.driving_direction == "CW") or 
         (car.left_dist >= 100 and car.driving_direction == "ACW"))): # right angle turn
        can_turn = False

        turn_radius = MIN_WALL_DIST + CAR_WIDTH/2
        if car.driving_direction == "CW":
            car.heading += 90 
            turn_radius = -turn_radius
            lower_ang, upper_ang = -15, 15
        elif car.driving_direction == "ACW":
            car.heading -= 90 
            lower_ang, upper_ang = -15, 15
        action_queue = deque([["turn", turn_radius, (upper_ang, lower_ang)]])

    else:
        action_queue = deque([["run"]])

    car.heading = confine_ang(car.heading)
    return action_queue

prev_time = 0
def excecute_action(action_queue: deque):
    """
    Perform action based on action queue.
    """

    completed = None
    global do_once
    global prev_time
    global just_done_green
    global just_done_red

    if action_queue:

        action = action_queue[0] 
        match action[0]:

            case "turn":
                turn_radius = action[1] 
                
                upper_ang, lower_ang = action[2]

                speed = 60 + min(40,40*abs(((confine_ang(car.heading - car.compass_direction))/90)))

                car.motor.speed(speed)
                car.turn(turn_radius)

                if is_ang_in_range(car.compass_direction, 
                                lower_bound=confine_ang(car.heading+lower_ang), 
                                upper_bound=confine_ang(car.heading+upper_ang)): 
                    completed = True

            case "turn avoid":
                turn_radius = action[1] 
                heading = action[2]

                upper_ang, lower_ang = 5, -5
                car.heading = heading

                speed = SPEED
                car.motor.speed(speed)
                car.turn(turn_radius)

                if is_ang_in_range(car.compass_direction, 
                                lower_bound=confine_ang(heading+lower_ang), 
                                upper_bound=confine_ang(heading+upper_ang)): 
                    completed = True
            
            case "run till":
                car.motor.speed(SPEED)
                car.servo.write(car.pid_straight((1,0,0),car.heading))

                if not do_once:
                    prev_time = time.time()
                    do_once = True

                required_time = action[1]

                if time.time() - prev_time >= required_time:
                    completed = True
                    do_once = False

            case "run": 
                car.motor.speed(SPEED)
                car.servo.write(car.pid_straight((1,0,0)))
                completed = True
                if green_sign.map_x != -999:
                    just_done_green = False
                if red_sign.map_x != -999:
                    just_done_red = False
        
    if completed:
        action_queue.popleft()

    return action_queue

def main():
    """
    Main program. Add your code here.
    """
    # variables
    global can_turn
    global just_done_red
    global just_done_green
    start = False 
    action_queue = deque()
    
    car.start_cam()

    print("Boot complete. \nPress the button to run.")
    # main program loop
    while True:

        car.read_sensors(True)
        car.get_frame()

        # Check for object and draw the bbox
        if green_sign.detect_sign(frame=car.frame, min_pixel_h=20, min_pixel_w=20):
            car.frame = green_sign.draw_bbox(car.frame, (0,255,0))
            # print(f"green y: {green_sign.map_y} green x: {green_sign.map_x} ")
            # cv2.imshow("mask green", green_sign.mask)

        if red_sign.detect_sign(frame=car.frame, min_pixel_h=20, min_pixel_w=20):
            car.frame = red_sign.draw_bbox(car.frame, (0,0,255))
        #     print(f"red y: {red_sign.y} red x: {red_sign.x}")
        #     # cv2.imshow("mask red", red_sign.mask)

        # cv2.imshow("Camera", car.frame)

        car.compass.set_home(car.read_button())

        if car.read_button():
            start = True
            print("Program started.")
            print("Running...")
            car.LED.rgb(100,100,100)
            time.sleep(0.2)

        if start: # Perform open challenge
            car.LED.rgb(0,200,0)

            action_queue = decide_action(action_queue)
            print(action_queue)
            print(car.heading)
            action_queue = excecute_action(action_queue) 
            
            if car.front_dist >= 110:
                can_turn = True

        else: 
            car.inactive()

        if cv2.waitKey(1) & 0xFF == ord('q'): #break out of loop if 'q' is pressed
            cv2.destroyAllWindows()
            break

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally: 
        # clean up any used resources (precaution)
        GPIO.cleanup()
        cv2.destroyAllWindows()