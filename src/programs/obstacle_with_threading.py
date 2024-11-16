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

# global variables
SPEED = 20              # in %
MIN_WALL_DIST = 30      # cm

def reset_driving():
    #reset
    car.servo.write(0)
    car.motor.speed(SPEED)

def confine_ang(ang):
    """Confine self.heading to 0 and 360 degrees"""
    ang %= 360
    if ang < 0: ang += 360
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

    #reset
    reset_driving()

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
    r = abs(r)
    print(f"x: {x}, y:{y}")
    if y < r*2: raise ValueError("End point too close")
    if y < 0: raise ValueError("Y should  be infront of car")

    cntr_2_cntr_length = sqrt((abs(x)-2*r)**2+y**2)                             # length of line btw the 2 arc centers
    alpha = degrees(acos(r/(cntr_2_cntr_length/2)))                        # angle between radius and c2c line
    phi = degrees(atan2(y,abs(x)-2*r))                               # angle from c2c line to +ve x axis
    theta = 180 - alpha - phi                                     # angle of tangent to normal
    tangent = (y - 2*r*sin(radians(theta))) / cos(radians(theta))  # length of tangent

    # if x is negative, turning direction is opposite
    if x < 0:
        return -theta, tangent
    else:
        return theta, tangent

prev_time = 0
cur_time = 0
def show_visuals():

    global prev_time
    global cur_time

    font = cv2.FONT_HERSHEY_SIMPLEX
    line = cv2.LINE_AA

    # print green bbox and coordinates
    if green_sign.x != -999:
        car.frame = green_sign.draw_bbox(car.frame, (0,255,0))
        text_pos = (50,camera["shape"][1] - 50)
        car.frame = cv2.putText(car.frame, f"({green_sign.map_x:.2f}, {green_sign.map_y:.2f})",
                                text_pos, font, 1, (0,255,0), 1, line)

    # print red bbox and coordinates
    if red_sign.x != -999:
        car.frame = red_sign.draw_bbox(car.frame, (0,0,255))
        text_pos = (camera["shape"][0] - 250,camera["shape"][1] - 50)
        car.frame = cv2.putText(car.frame, f"({red_sign.map_x:.2f},{red_sign.map_y:.2f})",
                                text_pos, font, 1, (0,0,255), 1, line)
        
    # if parking_lot.x != -999:
    #     car.frame = parking_lot.draw_bbox(car.frame, (255,51,255))
    #     text_pos = (200,camera["shape"][1] - 50)
    #     car.frame = cv2.putText(car.frame, f"({parking_lot.map_x:.2f},{parking_lot.map_y:.2f})",
    #                             text_pos, font, 1, (255,51,255), 1, line)

    prev_time = cur_time
    cur_time = time.time()
    fps = 1 / (cur_time-prev_time)
    car.frame = cv2.putText(car.frame, f"fps: {fps:.2f}",
                    (30,30), font, 1, (255,0,0), 1, line)
    
    cv2.imshow("Camera", car.frame)
    # cv2.imshow("Parking Lot", parking_lot.mask)
    cv2.imshow("Red", red_sign.mask)
    # cv2.imshow("Green", green_sign.mask)

def setup():
    car.start_cam()

def background_tasks():

    global exit_key_presesd

    try:
        while True:
            car.read_sensors(True)
            car.read_button()
            car.compass.set_home(car.read_button())

            car.get_frame()

            green_sign.detect_sign(frame=car.frame, min_pixel_h=20, min_pixel_w=40)
            red_sign.detect_sign(frame=car.frame, min_pixel_h=20, min_pixel_w=40)
            # parking_lot.detect_sign(frame=car.frame, min_pixel_h=20, min_pixel_w=20)

            # show_visuals()

            if cv2.waitKey(1) & 0xFF == ord('q'): #break out of loop if 'q' is pressed
                cv2.destroyAllWindows()
                break

    except Exception as e:
        print(f"Error in background tasks: {e}")


def main():

    car.driving_direction = "ACW"
    total_turns = 12

    start = False
    can_turn = True
    no_of_turns = 0
    start_pos = ()

    print("Boot complete. \nPress the button to run.")  

    while True:

        # End the round
        if (no_of_turns == total_turns 
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

            # # pass on left of green traffic sign
            # if (green_sign.map_x != -999 and green_sign.map_x < CAR_WIDTH/2
            # and (35 < green_sign.map_y < 50)):
            #     print(f"Passing on left. Green sign: {green_sign.map_x:.2f} cm, {green_sign.map_y:.2f}")

            #     car.LED.rgb(0,200,0)
            #     x = green_sign.map_x - green_sign.width/2 - CAR_WIDTH/2
            #     y = green_sign.map_y
            #     r = 16

            #     curve_to_point(r, x, y)
                
            #     drive_dist(green_sign.width+5, r)
                
            #     # curve back to middle
            #     arc(-16, confine_ang(car.heading+90), tol=5)
            #     arc(16, confine_ang(car.heading-90), tol=5)

            # # pass on right of red traffic sign
            # elif (red_sign.map_x != -999 and red_sign.map_x > -(CAR_WIDTH/2 + RED_SIGN["width"])
            # and (33 < red_sign.map_y < 60)):
            #     print(f"Passing on right. Red sign: {red_sign.map_x:.2f} cm, {red_sign.map_y:.2f}")
                
            #     car.LED.rgb(200,0,0)
            #     x = red_sign.map_x + red_sign.width/2 + CAR_WIDTH / 2
            #     y = red_sign.map_y
            #     r = 16

            #     curve_to_point(-r, x, y)
                
            #     drive_dist(5+red_sign.width+5, 20)
                
            #     # curve back to middle
            #     arc(16, confine_ang(car.heading-90), tol=5)
            #     arc(-16, confine_ang(car.heading+90), tol=5)

            # corner turn
            # el
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