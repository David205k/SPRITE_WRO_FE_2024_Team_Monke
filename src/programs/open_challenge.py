"""
This program is to be run for the WRO Future Engineers Open Challenge.
"""

from monke_hat import Car
from component_params import *

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

# initialise car object
car = Car.Car(
    camera=camera,
    wheelBase=wheelBase,
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

def main():
    """
    Main control loop of the program. Add your code here.
    """




    pass

if __name__ == "__main__":
    try:
        main()
    except Exception as E:
        print(f"Error: {E}")
    finally: 
        # clean up any used resources (precaution)
        GPIO.cleanup()
        cv2.destroyAllWindows()