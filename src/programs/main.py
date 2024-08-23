import RPi.GPIO as GPIO # use RPi library for controlling GPIO pins
import time
import math
import smbus
   
# modules for controlling components
import modules.Tb6612fngControl as Tb6612fng
import modules.PwmControl as PwmControl
import modules.RGBLEDControl as RGB
import modules.HMC5883LControl as HMC5883L
from gpiozero import DistanceSensor

from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import AngularServo

# modules for camera vision 
import cv2
from PIL import Image      
from picamera2 import Picamera2

GPIO.setwarnings(False) # turn off warnings for pins (if pins were previously used and not released properly there will be warnings)
GPIO.setmode(GPIO.BOARD) # pin name convention used is pin numbers on board
factory = PiGPIOFactory()

#variables
WHEELBASE = 0.12

# camera vision
picam2 = Picamera2()
picam2.preview_configuration.main.size=(1920,1000)
picam2.preview_configuration.main.format = 'RGB888'
picam2.start()

# initialise components  
compass = HMC5883L.compass(addr=0x1E)
servo = AngularServo(5, min_angle=-90, max_angle=90, min_pulse_width=0.0004, max_pulse_width=0.0026, pin_factory=factory) # pin is gpio pin
car = Tb6612fng.motor(stby=37, pwmA=35, ai1=36, ai2=40) 
LED = RGB.LED(red=8, blue=12, green=10)  

butPin = 16
GPIO.setup(butPin,GPIO.IN)

us2 = DistanceSensor(echo=27, trigger=22, max_distance=3) # pins are gpio pins
us3 = DistanceSensor(echo=10, trigger=9, max_distance=3) # pins are gpio pins
us4 = DistanceSensor(echo=6, trigger=13, max_distance=3) # pins are gpio pins

def main():
    while True:

        # get US distances in cm
        frontDist, leftDist, rightDist = us4.distance*100, us2.distance*100, us3.distance*100 
        
        angle = compass.getAngle()

        print(f"angle: {angle}, heading = {compass.heading}, startPos: {compass.startPos}buttonVal: {GPIO.Input(butPin)}")


if __name__ == "__main__":
    main()


