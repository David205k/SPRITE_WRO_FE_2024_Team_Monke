import sys
sys.path.append('/home/monke/WRO FE 2024 (Repository)/src/programs')

import cv2
from picamera2 import Picamera2 
from math import *
import time
import RPi.GPIO as GPIO                         # use RPi library for controlling GPIO pins
from gpiozero.pins.pigpio import PiGPIOFactory

from gpiozero import DistanceSensor
from PiicoDev_VL53L1X import PiicoDev_VL53L1X

import modules.monke_hat.Servo_control as MyServo
import modules.monke_hat.RGB_LED_control as RGB
import modules.monke_hat.LIS3MDL_control as LIS3MDL
import modules.monke_hat.Tb6612fng_control as Tb6612fng
import modules.monke_hat.PID as PID
import modules.monke_hat.VL53L1X_control as VL51L1X
import modules.monke_hat.HMC5883L_control as HMC5883L

from robot_config import * 
from helper_functions import *


factory = PiGPIOFactory()

GPIO.setmode(GPIO.BCM) 

class Car: 
    """
    Class for controlling team Monke's robot
    """

    def __init__(self):

        # initialise components  
        #---------------------------------------------------------------------------------------------
        self.servo = MyServo.Servo(servo["pin"], servo["start"], servo["offset"], servo["min"], servo["max"])
        self.motor = Tb6612fng.Motor(mDrvr["stby"], mDrvr["pwmA"], mDrvr["ai1"], mDrvr["ai2"]) 
        self.LED = RGB.LED(rgb["red"], rgb["blue"], rgb["green"])  

        # initialise compass 
        self.compass = self.check_component_connection(HMC5883L.Compass, "Compass", Exception, (255,255,0))
        # self.compass = self.check_connection(LIS3MDL.Compass, "Compass", ValueError, (255,255,0))

        # initialise pushbutton
        # two pins are connected to the start button as safeguard
        GPIO.setup(pb[1],GPIO.IN)        
        GPIO.setup(pb[2],GPIO.IN)

        self.us_front = DistanceSensor(echo=us_front["echo"], trigger=us_front["trig"], max_distance=3, pin_factory=factory)
        
        self.tof_manager = VL51L1X.tof_manager((tof_left, tof_right))
         
        self.compass_direction = 0
        self.front_dist = 0
        self.left_dist = 0
        self.right_dist = 0
        self.back_dist = 0
        self.side_diff = 0
        self.but_press = False

        self.detection_zones = {}

        self.heading = 0
        self.driving_direction = "CW"

        self.PID_controller = None

    def check_component_connection(self, connect_function, device, error, LED_colour, *args, **kwargs):
        """
        Function to catch the error where a component is disconected.

        Prevents code from being stopped. Prints error message and turns on LED.

        Parameters:
        -----------
        connect_function:
            Function to connect to module which may produce an error.
        device:
            The component to connect to. Will display this name in the success and error msg
        LED_colour:
            Colour to display on the LED when no connection
        *args:
            positional arguments for function
        **kwargs:
            keyword arguments for function
        """
        n = 0
        while True:
            n+=1

            try:
                return_object = connect_function(*args, **kwargs)
            except error:
                print(f"Error. Unable to connect to {device}")
                r,g,b = LED_colour
                self.LED.rgb(r,g,b)
                continue
        
            if n > 1:
                print(f"Successful connection to {device}")
                self.LED.off()
            
            return return_object

    def straight(self, speed: float):
        """
        Move straight

        Parameters
        ----------
        speed: float
            speed when moving straight (-100 to 100.0)
        """
        self.servo.write(0)
        self.motor.speed(speed)

    def stop(self):
        """Stop the car and bring servo to 0"""
        self.servo.write(0)
        self.motor.speed(0)

    def turn(self, radius):
        """
        Function to write servo angle for a specified turn radius.

        Required wheel axis angle is derived using car's wheelbase, turn radius, and trigo.
        Based on the car's steering mechanism, the required
        servo position is calculated and written to the servo.

        Parameters
        ----------
        radius: float
            +ve => turn ACW; -ve => turn CW
        """

        angled_link_ang_offset = radians(ANGLED_LINK_ANG_OFFSET)

        sign = radius/abs(radius)
        radius = max(abs(radius), WHEELBASE) # set minimum turn radius to wheelbase

        theta = asin(WHEELBASE/radius)
        link_ang = angled_link_ang_offset-theta

        # circle c3
        c3_x = ANGLED_LINK*cos(link_ang)
        c3_y = ANGLED_LINK*sin(link_ang)

        # circle c2
        c2_x = DIST_BTW_PIVOTS
        c2_y = 0

        d = sqrt((c3_x-c2_x)**2+(c3_y-c2_y)**2)     # dist btw c2 and c3
        c3_ang_to_c2 = atan(abs(c2_y-c3_y)/abs(c2_x-c3_x))  # angle of str line to x axis from centre of c2 to c3

        x, y1, y2, alpha = get_circle_intersection(YOKE,ANGLED_LINK,d) 

        yoke_ang = alpha - c3_ang_to_c2

        mid_x_new = c3_x + (cos(yoke_ang)*YOKE)/2
        mid_y_new = c3_y + sin(yoke_ang)*YOKE

        delta_x = mid_x_new-SERVO_CTR[0]
        delta_y = mid_y_new-SERVO_CTR[1]
        delta_ang = atan(abs(delta_x)/abs(delta_y))

        delta_ang = delta_ang*sign
        self.servo.write(round(degrees(delta_ang)))     

    # old robot turn function
    def turn_old(self, radius): 
        """
        Drive car in circle with specified radius

        Parameters
        ----------
        radius: float  
            Radius of circle in cm. (radius >= wheelbase)
            +ve => ACW
            -ve => CW
        """
        if -WHEELBASE <= radius <= WHEELBASE:
            radius = WHEELBASE * radius/abs(radius)
        ang = round(degrees(atan(WHEELBASE/(radius))))
        self.servo.write(ang)
    
    i = 0
    def pid_straight(self, pid: tuple, verbose: bool = False) -> float:
        """
        Returns servo angle to keep robot algined with intended direction.

        Parameters
        ----------
        pid: tuple
            pid = (kp, ki, kd),  kp, ki, kd: float
        intended_direction: float
            Angle from 0-360 w.r.t compass angle to keep the robot aligned to
        verbose: bool
            True: print angle adjustment False: no output
        angle_adj:
            angle to write to servo
        """
        kp, kd, ki = pid
        if self.i == 0:
            self.PID_controller = PID.PID(kp, ki, kd)
            self.i = 1

        ang_diff = self.compass_direction - self.heading
        if ang_diff < -180:
            ang_diff += 360
        elif ang_diff > 180:
            ang_diff -= 360

        angle_adj = self.PID_controller.PID_control(ang_diff)

        if verbose:
            print(angle_adj)

        return angle_adj

    def inactive(self):
        """
        Off state for the car.

        LED => off, Motor => 0 speed, Servo => 0 degrees
        """

        self.stop()
        self.LED.off()

    def start_cam(self):
        """Initialise the car's camera"""
        self.picam2 = Picamera2()
        self.picam2.preview_configuration.main.size=(1920,1000)
        self.picam2.preview_configuration.main.format = 'RGB888'
        self.picam2.start()

    prev_time = 0
    cur_time = 0
    def get_frame(self, rotate: bool =False, fps: bool = True):
        """
        Reads video frames from video capture and returns manipulated frames.

        Parameters
        ----------
        rotate: bool
            True: rotate frame 180 degs | False: no rotation
        fps: bool
            True: Show FPS on screen | False: Don't show FPS
        """

        self.frame = self.picam2.capture_array()

        self.frame = cv2.resize(self.frame, camera["shape"]) # resize camera feed

        if rotate:
            self.frame = cv2.flip(self.frame, 0) # Flip vertically
            self.frame = cv2.flip(self.frame, 1) # Flip horizontally
    
        font, line = cv2.FONT_HERSHEY_SIMPLEX, cv2.LINE_AA

        # get fps
        self.prev_time = self.cur_time
        self.cur_time = time.time()
        fps = 1 / (self.cur_time-self.prev_time)
        self.frame = cv2.putText(self.frame, f"fps: {fps:.2f}",
                        (30,30), font, 1, (255,0,0), 1, line)

        return self.frame


    def read_button(self, verbose: bool = False) -> bool:
        """
        Reads button press and returns its status.

        If button is pressed, True is returned, else False 
        
        Parameters
        ----------
        verbose: bool
            True: prints out "button pressed" if button pressed | False: no output
        """
        
        if verbose:
            print("Button was pressed") 

        if (GPIO.input(pb[1]) and GPIO.input(pb[2])):
            self.but_press = True
        else:
            self.but_press = False
        return self.but_press

    def read_sensors(self, verbose: bool = False):
        """
        Read sensors and store their values in their respective instance variables

        Reads ultrasonic sensors and HMC5883L compass module. Values of sensors are returned

        Parameters
        ----------
        verbose: bool
            True: print sensor vals | False: no output
        """
        self.front_dist = round(self.us_front.distance*100)
        self.left_dist, self.right_dist = self.tof_manager.read_sensors()
        # self.left_dist = (self.tof_left.read() // 10) 
        # self.right_dist = (self.tof_right.read() // 10) 
        # self.back_dist = (self.tof_back.read() // 10) 
        self.side_diff = self.left_dist - self.right_dist
        self.compass_direction = round(self.check_component_connection(
            self.compass.get_angle, "Compass", Exception, (255, 255, 0)))

        if verbose:
            print(f"Front: {self.front_dist} Left: {self.left_dist} Right: {self.right_dist} Back: {self.back_dist} Compass: {self.compass_direction}")

        return self.front_dist, self.left_dist, self.right_dist, self.compass_direction
        
    def print_sensor_vals(self):
        """Print sensor readings to console"""

        self.read_sensors()
        print(f"Front: {self.front_dist} Left: {self.left_dist} Right: {self.right_dist} Back: {self.back_dist} Compass: {self.compass_direction}")

    def reset(self):
        self.inactive()
        self.compass.set_home()

        self.heading = 0