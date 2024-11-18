from modules.Traffic_sign.Traffic_sign import Traffic_sign
from modules.monke_hat.Car import Car
from parameters import *
from programs.robot_config import *
from helper_functions import *

from RPi import GPIO
import threading
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
parking_lot = Traffic_sign(PARKING_LOT)


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

def avoid_sign(sign:Traffic_sign, pass_on_side:str):

    x_within_dist = {"left": sign.map_x <= CAR_WIDTH/2,
                  "right": sign.map_x >= -CAR_WIDTH/2}
    
    side = 1 if pass_on_side == "left" else -1

    if (sign.have_sign and x_within_dist
        and 35 <= sign.map_y <= 100):

        print(f"Passing on {pass_on_side}. {sign.type} at {sign.map_x:.2f},{sign.map_y:.2f}")
        b,g,r = sign.bbox_colour
        car.LED.rgb(r,g,b)

        buffer = 5
        x = sign.map_x + (CAR_WIDTH/2 + sign.width/2 + buffer) * -side
        y = sign.map_y
        r = 16
        r = r * side
        
        print(f"Moving to {(x,y)}")

        # avoid object
        curve_to_point(r, x, y)

        # drive straight pass the object
        drive_dist(green_sign.width+5)
        
        # curve back to middle
        arc(-16*side, confine_ang(car.heading+90*side), tol=5)
        arc(16*side, confine_ang(car.heading-90*side), tol=5)

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
    
    # draw bbox and object coordinates
    green_sign.draw_bbox()
    red_sign.draw_bbox()
    parking_lot.draw_bbox()
    
    cv2.imshow("Camera", car.frame)
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

            green_sign.detect_sign(frame=car.frame)
            red_sign.detect_sign(frame=car.frame)
            # parking_lot.detect_sign(frame=car.frame)

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

    print("Boot complete. \nPress the button to run.")  

    while True:

        # End the round
        if (no_of_turns == TOTAL_TURNS 
        and (car.front_dist <= start_pos[0]+10)): # stop at start position
            start = False

        if car.but_press:
            start = True
            
            car.reset()     # reset all variables

            start_pos = (car.front_dist, car.left_dist, car.right_dist)  # store starting position

            print("Program started.")
            print("Running...")
            car.LED.rgb(100,100,100)
            time.sleep(0.2)

        if start:
            # Main code start here
            car.LED.rgb(0,0,200)

            diff = car.left_dist - car.right_dist
            
            if red_sign.have_sign:
                avoid_sign(red_sign, "right")
            elif green_sign.have_sign:
                avoid_sign(green_sign, "left")
            elif (car.front_dist <= 70 and can_turn and abs(car.side_diff) >= 10):
                print(f"Turning. Front: {car.front_dist:.0f} Left: {car.left_dist:.0f} Right {car.left_dist:.0f}")

                car.driving_direction = "ACW" if diff >= 0 else "CW"

                no_of_turns += 1
                can_turn = False

                turn_radius = 20

                lower_ang, upper_ang = -3, 3

                if car.driving_direction == "CW":
                    car.heading += 90 
                    turn_radius = -turn_radius
                elif car.driving_direction == "ACW":
                    car.heading -= 90 

                arc(turn_radius, car.heading, 
                speed=20 + min(10,10*abs(((confine_ang(car.heading-car.compass_direction))/90))),
                lower_tol=lower_ang, upper_tol=upper_ang)

                drive_dist(CAR_LENGTH/2,-20)

            # # avoid when too close to walls
            # elif car.left_dist <= 10 or car.right_dist <= 10:
            #     print("Avoiding")
            #     x = 30
            #     x = x if car.left_dist <= 10 else -x
            #     curve_to_point(16, x, 35)

            else:
                car.motor.speed(SPEED)
                car.servo.write(car.pid_straight((1,0,0)))

                if car.front_dist >= 75:
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
        # clean up any used resources (precaution)
        car.inactive()
        background_thread.join()
        GPIO.cleanup()
        cv2.destroyAllWindows()