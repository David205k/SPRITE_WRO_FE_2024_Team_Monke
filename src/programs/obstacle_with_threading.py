from modules.Traffic_sign.Traffic_sign import Traffic_sign
from modules.monke_hat import Car
from parameters import *
from component_params import *

from RPi import GPIO
import threading
import cv2
from math import *
import time

# objects
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
parking_lot = Traffic_sign(PARKING_LOT)


def reset_driving():
    #reset
    car.servo.write(0)
    car.motor.speed(SPEED)

def drive_dist(dist, speed=SPEED):
    car.motor.speed(speed)
    car.servo.write(car.pid_straight((1,0,0)))
    time.sleep(dist/(abs(speed)/100 * MAX_SPEED_CMS))

    #reset
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

def avoid_taffic_sign(sign:Traffic_sign, pass_on_side:str):

    x_within_dist = {"left": sign.map_x <= CAR_WIDTH/2,
                  "right": sign.map_x >= -CAR_WIDTH/2}

    if (sign.have_sign and x_within_dist
        and 35 <= sign.map_y <= 100):

        print(f"Passing on {pass_on_side}. {sign.colour} at {sign.map_x:.2f},{sign.map_y:.2f}")
        b,g,r = sign.bbox_colour
        car.LED.rgb(r,g,b)

        buffer = 5
        x = CAR_WIDTH/2 + sign.width/2 + buffer
        y = sign.map_y
        r = 16

        print(f"Moving to {(x,y)}")

        curve_to_point(r, x, y)
        
        drive_dist(green_sign.width+5)
        
        # curve back to middle
        arc(-16, confine_ang(car.heading+90), tol=5)
        arc(16, confine_ang(car.heading-90), tol=5)


    

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

            avoid_traffic_sign()
            if (car.front_dist <= 70 and can_turn  and
                  ((car.right_dist >= 100 and car.driving_direction == "CW") or 
                (car.left_dist >= 100 and car.driving_direction == "ACW"))):
                print(f"Turning. Front: {car.front_dist:.0f} Left: {car.left_dist:.0f} Right {car.left_dist:.0f}")

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