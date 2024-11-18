import sys
sys.path.append('/home/monke/WRO FE 2024 (Repository)/src/programs')

import RPi.GPIO as GPIO # use RPi library for controlling GPIO pins
from gpiozero.pins.pigpio import PiGPIOFactory

from gpiozero import DistanceSensor
from PiicoDev_VL53L1X import PiicoDev_VL53L1X

import modules.monke_hat.Servo_control as MyServo
import modules.monke_hat.Pwm_control as PWM
import modules.monke_hat.RGB_LED_control as RGB
# import modules.monke_hat.HMC5883L_control as HMC5883L
import modules.monke_hat.LIS3MDL_control as LIS3MDL
import modules.monke_hat.Tb6612fng_control as Tb6612fng
import modules.monke_hat.PID as PID
from parameters import * 

import cv2
from picamera2 import Picamera2
from math import *

factory = PiGPIOFactory()

GPIO.setwarnings(False) # turn off warnings for pins (if pins were previously used and not released properly there will be warnings)
# GPIO.setmode(GPIO.BOARD) # pin name convention used is pin numbers on board

class Car: 
    """
    Class for controlling team Monke's vehicle

    Methods
    -------
    straight(speed: float)
    stop()
    start_cam()
    """

    def __init__(self,
                 camera: dict,
                 servo: dict,
                 us_front: dict,
                 us_left: dict,
                 us_right: dict,
                 us_spare1: dict,
                 us_spare2: dict,
                 rgb: dict,
                 pb: dict,
                 mDrvr: dict,
                 robot_params: dict,
                 ):

        self._camera = camera
        self._servo = servo
        self._us_front = us_front
        self._us_left = us_left
        self._us_right = us_right
        self._us_spare1 = us_spare1
        self._us_spare2 = us_spare2
        self._rgb = rgb
        self._pb = pb
        self._mDrvr = mDrvr
        self._robot_params = robot_params

        # initialise components  
        #---------------------------------------------------------------------------------------------
        # initialise compass 
        # while True:  # try until it connects
        #     try:
        #         self.compass = HMC5883L.compass()
        #     except OSError:
        #         print("Unable to connect to compass")
        #         continue 
        #     print("Connection to compass successful!")
        #     break 

        self.compass = LIS3MDL.compass()

        # initialise pushbutton
        GPIO.setup(pb[1],GPIO.IN)        # two pins are connected to the start button as safeguard
        GPIO.setup(pb[2],GPIO.IN)

        self.servo = MyServo.Servo(servo["pin"], servo["start"], servo["offset"], servo["min"], servo["max"])
        self.motor = Tb6612fng.Motor(mDrvr["stby"], mDrvr["pwmA"], mDrvr["ai1"], mDrvr["ai2"]) 
        self.LED = RGB.LED(rgb["red"], rgb["blue"], rgb["green"])  

        # initialise ultrasonic sensors
        self.us_front = DistanceSensor(echo=us_front["echo"], trigger=us_front["trig"], max_distance=3, pin_factory=factory)
        
        left_shut = 11
        right_shut = 8
        GPIO.setup(left_shut, GPIO.OUT)
        GPIO.setup(right_shut, GPIO.OUT)
        
        GPIO.output(left_shut, GPIO.LOW)
        GPIO.output(right_shut, GPIO.LOW)

        GPIO.output(left_shut, GPIO.HIGH)
        self.tof_left = PiicoDev_VL53L1X( bus=1, sda=27, scl=28, freq = 400_000 )
        self.tof_left.change_addr(0x30)

        GPIO.output(right_shut, GPIO.HIGH)
        self.tof_right = PiicoDev_VL53L1X( bus=1, sda=27, scl=28, freq = 400_000)
        self.tof_right.change_addr(0x31)

        self.compass_direction = 0
        self.front_dist = 0
        self.left_dist = 0
        self.right_dist = 0
        self.side_diff = 0
        self.but_press = False

        self.heading = 0
        self.driving_direction = "CW"

        self.PID_controller = None

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
        ang = round(degrees(asin(WHEELBASE/radius)))
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
        # self.servo.write(angle_adj)

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
        self.picam2.preview_configuration.main.size=self._camera["shape"] #(1920,1000)
        self.picam2.preview_configuration.main.format = 'RGB888'
        self.picam2.start()

    def get_frame(self, rotate: bool =True):
        """
        Reads video frames from video capture and returns manipulated frames.

        Parameters
        ----------
        rotate: bool
            True: rotate frame 180 degs | False: no rotation
        """

        self.frame = self.picam2.capture_array()

        if rotate:
            self.frame = cv2.flip(self.frame, 0) # Flip vertically
            self.frame = cv2.flip(self.frame, 1) # Flip horizontally

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

        if (GPIO.input(self._pb[1]) and GPIO.input(self._pb[2])):
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
        self.left_dist = (self.tof_left.read() // 10) 
        self.right_dist = (self.tof_right.read() // 10) 
        self.side_diff = self.left_dist - self.right_dist
        self.compass_direction = round(self.compass.get_angle())

        if verbose:
            print(f"Front: {self.front_dist} Left: {self.left_dist} Right: {self.right_dist} Compass: {self.compass_direction}")

        return self.front_dist, self.left_dist, self.right_dist, self.compass_direction
        
    def print_sensor_vals(self):
        """Print sensor readings to console"""

        self.read_sensors()
        print(f"Front: {self.front_dist} Left: {self.left_dist} Right: {self.right_dist} Compass: {self.compass_direction}")

    def reset(self):
        self.inactive()
        self.compass.set_home()

        self.heading = 0