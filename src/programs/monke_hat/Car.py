import RPi.GPIO as GPIO # use RPi library for controlling GPIO pins
from gpiozero.pins.pigpio import PiGPIOFactory

from gpiozero import DistanceSensor

import monke_hat.ServoControl_gpiozero as MyServo
import monke_hat.PwmControl as PWM
import monke_hat.RGBLEDControl as RGB
import monke_hat.HMC5883LControl as HMC5883L
import monke_hat.Tb6612fngControl as Tb6612fng

# import cv2
# from picamera2 import Picamera2

GPIO.setwarnings(False) # turn off warnings for pins (if pins were previously used and not released properly there will be warnings)
GPIO.setmode(GPIO.BOARD) # pin name convention used is pin numbers on board
factory = PiGPIOFactory()

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
                 wheelBase: float,
                 servo: dict,
                 us_front: dict,
                 us_left: dict,
                 us_right: dict,
                 us_spare1: dict,
                 us_spare2: dict,
                 rgb: dict,
                 pb: dict,
                 mDrvr: dict
                 ):

        self._camera = camera
        self._WHEELBASE = wheelBase
        self._servo = servo
        self._us_front = us_front
        self._us_left = us_left
        self._us_right = us_right
        self._us_spare1 = us_spare1
        self._us_spare2 = us_spare2
        self._rgb = rgb
        self._pb = pb
        self._mDrvr = mDrvr

        # initialise components  
        #---------------------------------------------------------------------------------------------
        # initialise compass 
        while True:  # try until it connects
            try:
                self.compass = HMC5883L.compass()
            except OSError:
                print("Unable to connect to compass")
                continue 
            print("Connection to compass successful!")
            break 

        # initialise pushbutton
        GPIO.setup(pb[1],GPIO.IN)        # two pins are connected to the start button as safeguard
        GPIO.setup(pb[2],GPIO.IN)

        self.servo = MyServo.Servo(servo["pin"], servo["start"], servo["offset"], servo["min"], servo["max"])
        self.motor = Tb6612fng.Motor(mDrvr["stby"], mDrvr["pwmA"], mDrvr["ai1"], mDrvr["ai2"]) 
        self.LED = RGB.LED(rgb["red"], rgb["blue"], rgb["green"])  

        # initialise ultrasonic sensors
        self.us_front = DistanceSensor(echo=us_front["echo"], trigger=us_front["trig"], max_distance=3, pin_factory=factory)
        self.us_left = DistanceSensor(echo=us_left["echo"], trigger=us_left["trig"], max_distance=3, pin_factory=factory)
        self.us_right = DistanceSensor(echo=us_right["echo"], trigger=us_right["trig"], max_distance=3, pin_factory=factory)
        # self.us_spare1 = DistanceSensor(echo=us_spare1["echo"], trigger=us_spare1["trig"], max_distance=3, pin_factory=factory)
        # self.us_spare2 = DistanceSensor(echo=us_spare2["echo"], trigger=us_spare2["trig"], max_distance=3, pin_factory=factory)  

        self.compass_direction = 0
        self.front_dist = 0
        self.left_dist = 0
        self.right_dist = 0
    
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
        """
        Stop the car and bring servo to 0
        """
        self.servo.write(0)
        self.motor.speed(0)

    # def start_cam(self):
    #     """
    #     Initialise the car's camera
    #     """
    #     self.picam2 = Picamera2()
    #     self.picam2.preview_configuration.main.size=(1920,1000)
    #     self.picam2.preview_configuration.main.format = 'RGB888'
    #     self.picam2.start()
    #     self.cap = cv2.VideoCapture(0)

    # def get_frame(self, rotate: bool =True):
    #     """
    #     Reads video frames from video capture and returns manipulated frames.

    #     Parameters
    #     ----------
    #     rotate: bool
    #         True: rotate frame 180 degs | False: no rotation
    #     """

    #     self.frame = self.picam2.capture_array()

    #     if rotate:
    #         self.frame = cv2.flip(self.frame, 0) # Flip vertically
    #         self.frame = cv2.flip(self.frame, 1) # Flip horizontally

    #     self.frame = cv2.resize(self.frame, (self._camera["shape"]))

    #     return self.frame
    
    def read_button(self, verbose: bool = False) -> bool:
        """
        Reads button press and returns its status.

        If button is pressed, True is returned, else False 
        
        Parameters
        ----------
        verbose: bool
            True: prints out "button pressed" if button pressed | False: no output
        """
        
        if (GPIO.input(self._pb[1]) and GPIO.input(self._pb[2])):
            print("Button was pressed") 
            return True
        else:
            return False

    def read_sensors(self, verbose: bool = False):
        """
        Read sensors and store their values in their respective instance variables

        Reads ultrasonic sensors and HMC5883L compass module

        Parameters
        ----------
        verbose: bool
            True: print sensor vals | False: no output
        """

        self.front_dist = round(self.us_front.distance*100, 2)
        self.left_dist = round(self.us_left.distance*100, 2)
        self.right_dist = round(self.us_right.distance*100, 2)
        
        self.compass_direction = self.compass.get_angle()

        if verbose:
            print(f"Front: {self.front_dist} Left: {self.left_dist} Right: {self.right_dist} Compass: {self.compass_direction}")

        return self.front_dist, self.left_dist, self.right_dist, self.compass_direction
        
    def print_sensor_vals(self):
        """
        Print sensor readings to console
        """
        self.read_sensors()

        print(f"Front: {self.front_dist} Left: {self.left_dist} Right: {self.right_dist} Compass: {self.compass_direction}")
