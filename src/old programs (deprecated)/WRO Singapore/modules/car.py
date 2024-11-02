import RPi.GPIO as GPIO # use RPi library for controlling GPIO pins
import time
import math
from gpiozero.pins.pigpio import PiGPIOFactory

# modules for controlling components
import modules.Tb6612fngControl as Tb6612fng
import modules.RGBLEDControl as RGB
import modules.HMC5883LControl as HMC5883L
import modules.ServoControl_gpiozero as myservo
from gpiozero import DistanceSensor

GPIO.setwarnings(False) # turn off warnings for pins (if pins were previously used and not released properly there will be warnings)
GPIO.setmode(GPIO.BOARD) # pin name convention used is pin numbers on board
factory = PiGPIOFactory()

class car()






#variables
WHEELBASE = 12          # vehicle wheelbase in cm
TOTALROUNDS = 3         # total number of rounds to be completed
compassDirection = 0    
drivingDirection = "CW" # round driving direction

# initialise components  
while True:
    try:
        compass = HMC5883L.compass(addr=0x1E)
    except OSError:
        print("Trying to connect to compass...")
        continue 
    print("Connection to compass successful!")
    break 

servo = myservo.myServo(gpioPin=5, startPos=0, offset=-13, minAng=-70, maxAng=70)
car = Tb6612fng.motor(stby=37, pwmA=35, ai1=36, ai2=40) 
LED = RGB.LED(red=8, blue=12, green=10)  

us2 = DistanceSensor(echo=27, trigger=22, max_distance=3, pin_factory=factory) # pins are gpio pins
us3 = DistanceSensor(echo=10, trigger=9, max_distance=3, pin_factory=factory) # pins are gpio pins
us4 = DistanceSensor(echo=6, trigger=13, max_distance=3, pin_factory=factory) # pins are gpio pins

startBut1 = 16
startBut2 = 18
GPIO.setup(startBut1,GPIO.IN)
GPIO.setup(startBut2,GPIO.IN)