"""
Description:
This program is used for our robot in the WRO FE obstacle challenge.

Usage:
- Specify robot configurations in robot_config.py
- Specify key operational values in parameters_obs.py
- During competition:
    1. Add execute script to crontab:
        - crontab -e
        - Add "@reboot python /path/to/code/obstacle_with_threading.py
        - Save
        - reboot
    2. Reboot Pi
    3. Wait for LED to turn white
    4. Press start button to start program
        - Ensure servo switch and motor switch are activated

Key Features:
- Reads sensors and detects obstacles in background()

Key Notes:
- comment "show_visuals()" in background tasks to improve processing speed

"""

from modules.object_detection.Traffic_sign import Traffic_sign
from modules.object_detection.Corner_line import Line
from modules.monke_hat.Car import Car
from parameters_obs import *
from robot_config import *
from helper_functions import *

from RPi import GPIO
import threading
import cv2
from math import *
import time
import zmq

import pickle

# Create a ZeroMQ PUSH socket
context = zmq.Context()
socket = context.socket(zmq.PUSH)
socket.bind("tcp://127.0.0.1:5555")  # Bind to a port

# declare objects
car = Car()

green_sign = Traffic_sign(GREEN_SIGN)
red_sign = Traffic_sign(RED_SIGN)
parking_lot = Traffic_sign(PARKING_LOT)

blue_line = Line(BLUE_LINE)
orange_line = Line(ORANGE_LINE)

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
            upper = car.heading-lower_tol
            lower = car.heading-upper_tol
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

def avoid_sign(sign:Traffic_sign, pass_on_side:str):

    x_within_dist = {"left": sign.map_x <= CAR_WIDTH/2,
                  "right": sign.map_x >= -CAR_WIDTH/2}
    
    side = 1 if pass_on_side == "left" else -1

    if (sign.have_sign and x_within_dist
        and 35 <= sign.map_y <= 100):

        print(f"Passing on {pass_on_side}. {sign.type} at {sign.map_x:.2f},{sign.map_y:.2f}")
        b,g,r = sign.bbox_colour
        car.LED.rgb(r,g,b)

        buffer = 3
        x = sign.map_x + (CAR_WIDTH/2 + sign.width/2 + buffer) * -side
        y = sign.map_y
        r = 20
        r = r * side
        
        print(f"Moving to {(x,y)}")

        # avoid object
        curve_to_point(r, x, y)

        # drive straight pass the object
        drive_dist(green_sign.width+5)
        
        # curve back to middle
        arc(-20*side, confine_ang(car.heading+90*side), tol=5)
        arc(20*side, confine_ang(car.heading-90*side), tol=5)

def show_visuals():
    
    # draw bbox and object coordinates
    car.frame = green_sign.draw_bbox(car.frame)
    car.frame = red_sign.draw_bbox(car.frame)
    car.frame = parking_lot.draw_bbox(car.frame)
    car.frame = blue_line.draw_line(car.frame)
    car.frame = orange_line.draw_line(car.frame)

    # draw zones
    for zone in (LINE_ZONE, SIGN_ZONE):
        cv2.rectangle(car.frame, (zone[:2]), (zone[0]+zone[2], zone[1]+zone[3]), 
                        (100,200,250), 1)

    cv2.imshow("Camera", car.frame)
    # cv2.imshow("Detection zone", detection_zone)
    # cv2.imshow("Orange mask", orange_line.mask) 
    # cv2.imshow("Blue mask", orange_line.mask) 
    # cv2.imshow("Parking Lot", parking_lot.mask)
    # cv2.imshow("Red", red_sign.mask)
    # cv2.imshow("Green", green_sign.mask)

def setup():
    car.start_cam()

def background_tasks():
    try:
        while True:
            car.read_sensors(True)
            car.read_button()
            car.compass.set_home(car.read_button())

            car.get_frame()
            # slice detection_zones
            car.detection_zones["lines"] = car.frame[LINE_ZONE[1]:LINE_ZONE[1]+LINE_ZONE[3], 
                                                  LINE_ZONE[0]:LINE_ZONE[0]+LINE_ZONE[2]]
            car.detection_zones["signs"] = car.frame[SIGN_ZONE[1]:SIGN_ZONE[1]+SIGN_ZONE[3], 
                                        SIGN_ZONE[0]:SIGN_ZONE[0]+SIGN_ZONE[2]]

            blue_line.detect_line(car.detection_zones["lines"])
            orange_line.detect_line(car.detection_zones["lines"])

            car_ang_from_normal = get_angular_diff(car.heading, car.compass_direction)
            # print(car_ang_from_normal)
            green_sign.detect_sign(car.detection_zones["signs"], car_ang_from_normal)
            red_sign.detect_sign(car.detection_zones["signs"], car_ang_from_normal)
            # parking_lot.detect_sign(frame=car.frame)

            # show_visuals()

            # socket.send(pickle.dumps({"front":car.front_dist, "left":car.left_dist, "right":car.right_dist,"compass": car.compass_direction, "heading": car.heading}))

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
    car.LED.rgb(50,50,50)
    while True:

        # End the round
        if (no_of_turns == TOTAL_TURNS 
        and (car.front_dist <= start_pos[0]+10)): # stop at start position
            start = False
            print("Run finished.")

        if car.but_press:
            start = True
            can_turn = True
            no_of_turns = 0
            turn_time = 0

            car.reset()  # reset all variables

            start_pos = (car.front_dist, car.left_dist, car.right_dist)  # store starting position

            print("Program started.")
            print("Running...")
            car.LED.rgb(100,100,100)
            time.sleep(0.2)

        if start:
            # Main code start here
            car.LED.rgb(10,50,10)

            diff = car.left_dist - car.right_dist
            
            # if red_sign.have_sign:
            #     avoid_sign(red_sign, "right")
            # elif green_sign.have_sign:
            #     avoid_sign(green_sign, "left")
            # el
            if (car.front_dist <= turn_dist-FRONT_2_TOF_LEN and can_turn and (car.left_dist>=100 or car.right_dist>=100)):
                print(f"Turning. Front: {car.front_dist:.0f} Left: {car.left_dist:.0f} Right {car.left_dist:.0f}")

                turn_time = time.time()
                no_of_turns += 1
                can_turn = False
                turn_radius = MIN_WALL_DIST

                if no_of_turns == 1:
                    # if orange_line.have_line and blue_line.have_line and abs(blue_line.angle-orange_line.angle) > 10:
                    #     print("blue line: ", blue_line.angle)
                    #     print("orange line: ", orange_line.angle)
                    #     if orange_line.angle > blue_line.angle:
                    #         car.driving_direction = "ACW"
                    #     else:
                    #         car.driving_direction = "CW"
                    # else:
                        car.driving_direction = "ACW" if diff >= 0 else "CW"
                        print(f"Driving_direction: {car.driving_direction}")

                # turn back to original position
                if no_of_turns == 12:
                    turn_radius = start_pos[1] if car.driving_direction == "CW" else start_pos[2]

                if car.driving_direction == "CW":
                    car.LED.rgb(255,100,5)
                    car.heading += 90 
                    turn_radius = -turn_radius
                elif car.driving_direction == "ACW":
                    car.LED.rgb(5,100,255)
                    car.heading -= 90 

                arc(turn_radius, car.heading, SPEED, lower_tol=-3, upper_tol=3)

            # elif ((car.left_dist <= 10 and car.driving_direction == "ACW") or 
            # (car.right_dist <= 10 and car.driving_direction == "CW") and car.front_dist >= turn_dist):
            #     car.LED.rgb(100,100,100)

            #     y = 40
            #     if car.driving_direction == "ACW":
            #         x = 10 - car.left_dist 
            #         r = -20 
            #     else:
            #         x = 10 - car.right_dist
            #         r = 20 
            #     curve_to_point(r,x,y)

            else:
                car.motor.speed(SPEED)
                car.servo.write(car.pid_straight((1,0,0)))

                if time.time() - turn_time >= 5 and car.front_dist >= 70:
                    can_turn = True

        else:
            car.inactive()

if __name__=="__main__":

    try:
        setup() 
        background_thread = threading.Thread(target=background_tasks, daemon=True)
        background_thread.start()
        try:
            main()
        except Exception as E:
            print(f"Error in main: {E}")
    except KeyboardInterrupt:
        pass
    finally: 
        # clean up any used resources (pre-caution)
        car.inactive()
        background_thread.join()
        GPIO.cleanup()
        cv2.destroyAllWindows()