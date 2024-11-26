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

from modules.object_detection.Corner_line import Line
from modules.monke_hat.Car import Car
from modules.monke_hat.PID import PID
import parameters_open as open
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

blue_line = Line(open.BLUE_LINE, open.LINE_ZONE)
orange_line = Line(open.ORANGE_LINE, open.LINE_ZONE)
edge = Line(open.WALLS, open.WALL_ZONE)

def reset_driving():
    #reset servo angle and motor speed to default
    car.servo.write(0)
    car.motor.speed(open.SPEED)

def drive_dist(dist, speed=open.SPEED):
    # drive a specified distance for a certain speed
    car.motor.speed(speed)
    car.servo.write(car.pid_straight((1,0,0)))
    time.sleep(dist/(abs(speed)/20 * MAX_SPEED_CMS))

    reset_driving()

def arc(radius:float, heading:float, speed:float=open.SPEED, tol:float = None, lower_tol:float = 0,  upper_tol:float = 0):

    car.heading = confine_ang(heading) # added

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

def curve_to_point(radius:float, x:float, y:float, speed=open.SPEED):
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
    print(f"    Curve to point, theta: {theta}, tan dist: {tan_dist}")

    # first arc
    arc(radius,heading=confine_ang(car.heading+theta),speed=speed,lower_tol=-5,upper_tol=5)

    # tangent
    drive_dist(tan_dist, speed)

    # returning arc
    arc(-radius,heading=confine_ang(car.heading-theta),speed=speed,lower_tol=-5,upper_tol=5)

    return theta, tan_dist

def show_visuals():
    
    # draw bbox and object coordinates
    car.frame = blue_line.draw_line(car.frame)
    car.frame = orange_line.draw_line(car.frame)
    car.frame = edge.draw_line(car.frame)

    # draw zones
    for zone in (open.LINE_ZONE, open.WALL_ZONE):
        cv2.rectangle(car.frame, (zone[:2]), (zone[0]+zone[2], zone[1]+zone[3]), 
                        (100,200,250), 1)

    cv2.imshow("Camera", car.frame)
    cv2.imshow("Orange mask", orange_line.mask) 
    cv2.imshow("Blue mask", blue_line.mask) 
    cv2.imshow("Walls", edge.mask)

def setup():
    car.start_cam()

angles = []
wall_angle = 0
def background_tasks():
    global angles
    global wall_angle
    try:
        while True:
            car.read_sensors(True)
            car.read_button()
            car.compass.set_home(car.read_button())

            car.get_frame()
            # slice detection_zones
            car.detection_zones["lines"] = car.frame[open.LINE_ZONE[1]:open.LINE_ZONE[1]+open.LINE_ZONE[3], 
                                                  open.LINE_ZONE[0]:open.LINE_ZONE[0]+open.LINE_ZONE[2]]

            car.detection_zones["walls"] = car.frame[open.WALL_ZONE[1]:open.WALL_ZONE[1]+open.WALL_ZONE[3], 
                                                  open.WALL_ZONE[0]:open.WALL_ZONE[0]+open.WALL_ZONE[2]]

            blue_line.detect_line(car.detection_zones["lines"])
            orange_line.detect_line(car.detection_zones["lines"])
            edge.detect_line(car.detection_zones["walls"])

            angles.append(edge.min_angle)
            wall_angle = round(moving_average(angles),3)
            
            # print(wall_angle)
            show_visuals()

            # socket.send(pickle.dumps({"front":car.front_dist, "left":car.left_dist, "right":car.right_dist,"compass": car.compass_direction, "heading": car.heading}))

            # print(f"Heading: {car.heading}, Compass: {car.compass_direction}")

            if cv2.waitKey(1) & 0xFF == ord('q'): #break out of loop if 'q' is pressed
                cv2.destroyAllWindows()
                break

    except Exception as e:
        print(f"Error in background tasks: {e}")

def main():
    global wall_angle
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
        if (no_of_turns >= open.TOTAL_TURNS 
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

            # curve_to_point(r,-x,y)
        if start:
            # Main code start here

            car.LED.rgb(255,0,255) # pink

            if no_of_turns == 0 and car.front_dist <=100: 
                print(blue_line.max_angle, orange_line.max_angle)
                car.driving_direction = "ACW" if blue_line.max_angle <= orange_line.max_angle else "CW"
            print(f"    Driving_direction: {car.driving_direction}")

            # corner turn
            if (car.front_dist <= 90 and can_turn and 
                (car.left_dist>=80 or car.right_dist>=80) and
                wall_angle <= 10):#or car.front_dist <=53
                print(f"Turning.\n    Front: {car.front_dist} Left: {car.left_dist} Right {car.left_dist} Compass {car.compass_direction}  Wall Angle {wall_angle}")
                
                no_of_turns += 1
                can_turn = False
                turn_radius = 20

                #if no_of_turns == 1:
                #    car.driving_direction = "ACW" if car.left_dist >= car.right_dist else "CW"
                print(f"    Driving_direction: {car.driving_direction}")


                if car.driving_direction == "CW":
                    car.LED.rgb(255,150,0) # orange
                    car.heading += 90 
                    turn_radius = -turn_radius
                elif car.driving_direction == "ACW":
                    car.LED.rgb(0,0,255) # blue
                    car.heading -= 90 

                car.heading = confine_ang(car.heading) # added
                print(f"    Car new heading: {car.heading}")
                print(f"    Number of turns: {no_of_turns}")
                print(f"    Round number: {no_of_turns//4+1}")
                print("\n")
                arc(turn_radius, car.heading, open.SPEED, lower_tol=-15, upper_tol=0)

    
            elif (car.left_dist <= 15# and 
                #   (wall_angle >= 15)
                # (car.left_dist <= 15 and car.driving_direction == "ACW") or 
                #   (car.right_dist <= 15 and car.driving_direction == "CW")
                ):
                car.LED.rgb(230,200,100) # yellow

                r = -18
                y = 2*abs(r)
                x = 10

                print(f"Avoiding wall left side.")
                print(f"    Front: {car.front_dist} Left: {car.left_dist} Right {car.left_dist} Compass {car.compass_direction} Wall Angle {wall_angle}")
                print(f"    r: {r}, x: {x}, y: {y}")
                curve_to_point(r,x,y,20)
            elif (car.right_dist <= 15 #and 
                #   (wall_angle >= 15)
                  ):
                # assuming distance is perpendicular distance of robot to wall
                car.LED.rgb(230,200,100) # yellow

                r = 18
                y = 2*abs(r)
                x = -10

                print(f"Avoiding wall right side.")
                print(f"    Front: {car.front_dist} Left: {car.left_dist} Right {car.left_dist} Compass {car.compass_direction} Wall Angle {wall_angle}")
                print(f"    r: {r}, x: {x}, y: {y}")
                curve_to_point(r,x,y,20)

            else:
                car.motor.speed(open.SPEED)
                car.servo.write(car.pid_straight((2,0,0)))

                if car.front_dist >= 110 and time.time() - turn_time >= 6:
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