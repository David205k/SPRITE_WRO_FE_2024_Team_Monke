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
from modules.monke_hat.PID import PID
import parameters_obs as obs
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

green_sign = Traffic_sign(obs.GREEN_SIGN, obs.SIGN_ZONE)
red_sign = Traffic_sign(obs.RED_SIGN, obs.SIGN_ZONE)
parking_lot = Traffic_sign(obs.PARKING_LOT, obs.SIGN_ZONE)

blue_line = Line(obs.BLUE_LINE, obs.LINE_ZONE)
orange_line = Line(obs.ORANGE_LINE, obs.LINE_ZONE)
edge = Line(obs.WALLS, obs.WALL_ZONE)

def reset_driving():
    #reset servo angle and motor speed to default
    car.servo.write(0)
    car.motor.speed(obs.SPEED)

def drive_dist(dist, speed=obs.SPEED):
    # drive a specified distance for a certain speed
    car.motor.speed(speed)
    car.servo.write(car.pid_straight((1,0,0)))
    time.sleep(dist/(abs(speed)/20 * MAX_SPEED_CMS))

    reset_driving()

def arc(radius:float, heading:float, speed:float=obs.SPEED, tol:float = None, lower_tol:float = 0,  upper_tol:float = 0):

    car.heading = confine_ang(heading)

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

    tol = 4
    # first arc
    arc(radius,heading=confine_ang(car.heading+theta),tol=tol)

    # tangent
    drive_dist(tan_dist, obs.SPEED)

    # returning arc
    arc(-radius,heading=confine_ang(car.heading-theta),tol=tol)

    return theta, tan_dist

def avoid_sign(sign:Traffic_sign, pass_on_side:str):
    side = 1 if pass_on_side == "left" else -1

    buffer = 5

    if ((sign.map_x <= CAR_WIDTH/2 + sign.width//2 + buffer and pass_on_side == "left") or
        (sign.map_x >= -CAR_WIDTH/2 - sign.width//2 - buffer and pass_on_side == "right") and
        sign.have_sign):
        if sign.map_y <= 40:
            print(f"Reversing, {sign.type} sign too close.")
            print(f"    x,y: {sign.map_x:.2f},{sign.map_y:.2f}")
            drive_dist(10, -obs.SPEED)
        elif (sign.map_y <= 80):
        
            print(f"Avoiding {sign.type} sign.")
            print(f"    x,y: {sign.map_x:.2f},{sign.map_y:.2f}")
            b,g,r = sign.bbox_colour
            car.LED.rgb(r,g,b)
            
            buffer = 0
            if sign.type == "red":
                buffer = 5
            else:
                buffer = 0
            x = sign.map_x + (CAR_WIDTH/2 + sign.width/2 + buffer) * -side
            y = sign.map_y
            r = 20
            r = r * side
            print(f"Moving to {(x,y)}")
            # while ((sign.map_x <= CAR_WIDTH/2 + sign.width//2 + buffer and pass_on_side == "left") or
            # (sign.map_x >= -CAR_WIDTH/2  - sign.width//2 - buffer and pass_on_side == "right") and sign.have_sign):
            #     error = CAR_WIDTH/2 + sign.width//2 + buffer - sign.map_x if pass_on_side == "left" else -CAR_WIDTH/2 - sign.width//2 -buffer - sign.map_x
            #     Kp = 4
            #     ang = (error / 100) * 45 * Kp
            #     car.servo.write(ang)
            #     car.motor.speed(obs.SPEED)
            # print("Traffic sign avoided.")
            # reset_driving()
            # avoid object
            curve_to_point(r, x, y)

            # # drive straight pass the object
            drive_dist(green_sign.width+6)
            
            # # # curve back to middle
            arc(-20*side, confine_ang(car.heading+90*side), tol=3)
            arc(18*side, confine_ang(car.heading-90*side), tol=3)

            drive_dist(17, -obs.SPEED)

def show_visuals():
    
    # draw bbox and object coordinates
    car.frame = green_sign.draw_bbox(car.frame)
    car.frame = red_sign.draw_bbox(car.frame)
    car.frame = parking_lot.draw_bbox(car.frame)
    # car.frame = blue_line.draw_line(car.frame)
    # car.frame = orange_line.draw_line(car.frame)
    car.frame = edge.draw_line(car.frame)

    # draw zones
    for zone in (obs.LINE_ZONE, obs.SIGN_ZONE, obs.WALL_ZONE):
        cv2.rectangle(car.frame, (zone[:2]), (zone[0]+zone[2], zone[1]+zone[3]), 
                        (100,200,250), 1)

    cv2.imshow("Camera", car.frame)
    # cv2.imshow("Detection zone", detection_zone)
    # cv2.imshow("Orange mask", orange_line.mask) 
    # cv2.imshow("Blue mask", blue_line.mask) 
    # cv2.imshow("Red", red_sign.mask)
    # cv2.imshow("Green", green_sign.mask)
    # cv2.imshow("Parking Lot", parking_lot.mask)
    cv2.imshow("Walls", edge.mask)
    cv2.imshow("Canny edges", edge.edges)

def setup():
    car.start_cam()

min_angles =[]
min_wall_angle = 0

max_angles =[]
max_wall_angle = 0

def background_tasks():
    global min_angles
    global min_wall_angle
    global max_angles
    global max_wall_angle
    global max_line
    try:
        while True:
            car.read_sensors()
            car.read_button()
            car.compass.set_home(car.read_button())

            car.get_frame()
            # slice detection_zones
            # car.detection_zones["lines"] = car.frame[LINE_ZONE[1]:LINE_ZONE[1]+LINE_ZONE[3], 
            #                                       LINE_ZONE[0]:LINE_ZONE[0]+LINE_ZONE[2]]
            
            car.detection_zones["signs"] = car.frame[obs.SIGN_ZONE[1]:obs.SIGN_ZONE[1]+obs.SIGN_ZONE[3], 
                                        obs.SIGN_ZONE[0]:obs.SIGN_ZONE[0]+obs.SIGN_ZONE[2]]

            car.detection_zones["walls"] = car.frame[obs.WALL_ZONE[1]:obs.WALL_ZONE[1]+obs.WALL_ZONE[3], 
                                                  obs.WALL_ZONE[0]:obs.WALL_ZONE[0]+obs.WALL_ZONE[2]]

            # blue_line.detect_line(car.detection_zones["lines"])
            # orange_line.detect_line(car.detection_zones["lines"])
            # edge.detect_line(car.detection_zones["walls"])

            # min_angles.append(edge.min_angle)
            # min_wall_angle = round(moving_average(min_angles),3)
            # max_angles.append(edge.max_angle)
            # max_wall_angle = round(moving_average(max_angles),3)
            # max_lines = edge.min_line

            # print("min wall angle: ", min_wall_angle)
            # print("max wall angle: ", max_wall_angle)

            ang_offset = get_angular_diff(car.heading, car.compass_direction)
            green_sign.detect_sign(car.detection_zones["signs"], ang_offset)
            red_sign.detect_sign(car.detection_zones["signs"], ang_offset)
            parking_lot.detect_sign(car.frame, ang_offset)

            # socket.send(pickle.dumps({"front":car.front_dist, "left":car.left_dist, "right":car.right_dist,"compass": car.compass_direction, "heading": car.heading}))

            if cv2.waitKey(1) & 0xFF == ord('q'): #break out of loop if 'q' is pressed
                cv2.destroyAllWindows()
                break


    except Exception as e:
        print(f"Error in background tasks: {e}")

def main():

    global min_wall_angle
    start = False
    can_turn = True
    no_of_turns = 0
    start_pos = ()
    turn_time = 0
    start_time = 0
    end_time = 0

    print("Boot complete. \nPress the button to run.")  
    
    while True:

        # End the round at start position
        if (no_of_turns == obs.TOTAL_TURNS 
        and (start_pos[0]-10 <= car.front_dist <= start_pos[0]+10) # stop at starting position
        # and is_ang_in_range(car.compass_direction, start_pos[3]-10, start_pos[3]+10) # stop at starting angle
        ): 
            start = False
            print("Run finished.")
            end_time = time.time()
            duration = end_time - start_time
            print(f"Run duration: {duration//60} mins and {round(duration%60)} secs")
            break

        if car.but_press:
            start = True

            car.reset()  # reset all variables

            car.read_sensors()
            start_pos = (car.front_dist, car.left_dist, car.right_dist, car.compass_direction)  # store starting position
            print(f"Starting position: {start_pos} - stored")

            print("Program started.")
            print("Running...")
            car.LED.rgb(100,100,100)
            time.sleep(0.2)
            
            start_time = time.time()
        if start:
            # Main code start here

            car.LED.rgb(255,0,255) # pink
            if red_sign.have_sign:
                avoid_sign(red_sign, "right")
            elif green_sign.have_sign:
                avoid_sign(green_sign, "left")
            elif (car.front_dist <= 80 and can_turn and 
                (car.left_dist>=100 or car.right_dist>=100) and 
                # (not red_sign.have_sign) and (no+-t green_sign.have_sign) and
                is_ang_in_range(car.compass_direction, car.heading-10, car.heading+10) #and
                # wall_angle <= 10
                ):

                print(f"Turning.\n       Front: {car.front_dist} Left: {car.left_dist} Right {car.left_dist} Compass {car.compass_direction}")
                
                no_of_turns += 1
                can_turn = False
                turn_radius = 20

                if no_of_turns == 1:
                    car.driving_direction = "ACW" if car.left_dist >= car.right_dist else "CW"
                print(f"    Driving_direction: {car.driving_direction}")

                if car.driving_direction == "CW":
                    car.LED.rgb(255,150,0) # orange
                    car.heading += 90 
                    turn_radius = -turn_radius
                elif car.driving_direction == "ACW":
                    car.LED.rgb(0,0,255) # blue
                    car.heading -= 90 

                car.heading = confine_ang(car.heading)
                print(f"    Car new heading: {car.heading}")
                print(f"    Number of turns: {no_of_turns}")
                print(f"    Round number: {no_of_turns//4+1}")
                print("\n")
                arc(turn_radius, car.heading, obs.SPEED, lower_tol=-20, upper_tol=0)

            # elif ((car.left_dist <= 10 and car.driving_direction == "ACW") or 
            # (car.right_dist <= 10 and car.driving_direction == "CW") and car.front_dist >= turn_dist):
            #     car.LED.rgb(230,200,100) # yellow

            #     y = 40
            #     if car.driving_direction == "ACW":
            #         x = 20 - car.left_dist 
            #         r = -20 
            #     else:
            #         x = 20 - car.right_dist
            #         r = 20 
            #     curve_to_point(r,x,y)

            else:
                car.motor.speed(obs.SPEED)
                car.servo.write(car.pid_straight((3,0,0)))

                if car.front_dist >= 100 and time.time() - turn_time >= 10:
                    can_turn = True
        else:
            car.inactive()

if __name__=="__main__":

    try:
        setup() 

        background_thread = threading.Thread(target=background_tasks, daemon=True)
        background_thread.start() # run background tasks

        try:
            main()
        except Exception as E: # catch exception in main 
            print(f"Error in main: {E}")

    except KeyboardInterrupt:

        print("Stopping program")

    finally: 
        # clean up any used resources (pre-caution)
        car.inactive()
        background_thread.join()
        GPIO.cleanup()
        cv2.destroyAllWindows()