import RPi.GPIO as GPIO # use RPi library for controlling GPIO pins
import time
import math
from gpiozero.pins.pigpio import PiGPIOFactory
from pynput import keyboard

# modules for controlling components
import modules.Tb6612fngControl as Tb6612fng
import modules.PwmControl as PwmControl
import modules.RGBLEDControl as RGB
import modules.HMC5883LControl as HMC5883L
from gpiozero import DistanceSensor
import modules.ServoControl_gpiozero as myservo


# modules for camera vision 
import cv2
from PIL import Image      
from picamera2 import Picamera2

GPIO.setwarnings(False) # turn off warnings for pins (if pins were previously used and not released properly there will be warnings)
GPIO.setmode(GPIO.BOARD) # pin name convention used is pin numbers on board
factory = PiGPIOFactory()

#variables
WHEELBASE = 12      # vehicle wheelbase in cm
TOTALROUNDS = 3
compassDirection = 0

picam2 = Picamera2()
picam2.preview_configuration.main.size=(1920,1000)
picam2.preview_configuration.main.format = 'RGB888'
picam2.start()

# initialise components  
try: 
    compass = HMC5883L.compass(addr=0x1E)
except OSError:

    while True:

        print("Trying to connect to compass...")
        try:
            compass = HMC5883L.compass(addr=0x1E)
        except OSError:
            continue 
        print("Connection successful!")
        break 

servo = myservo.myServo(gpioPin=5, startPos=0, minAng=-70, maxAng=70)
car = Tb6612fng.motor(stby=37, pwmA=35, ai1=36, ai2=40) 
LED = RGB.LED(red=8, blue=12, green=10)  

us2 = DistanceSensor(echo=27, trigger=22, max_distance=3, pin_factory=factory) # pins are gpio pins
us3 = DistanceSensor(echo=10, trigger=9, max_distance=3, pin_factory=factory) # pins are gpio pins
us4 = DistanceSensor(echo=6, trigger=13, max_distance=3, pin_factory=factory) # pins are gpio pins

startBut1 = 16
startBut2 = 18
#stopBut = 32
GPIO.setup(startBut1,GPIO.IN)
GPIO.setup(startBut2,GPIO.IN)
#GPIO.setup(stopBut,GPIO.IN)

# radius in cm
def turn(radius, headingDirection, direction): 

    tolerance = 5

    angleDiff = getAngularDiff(headingDirection, compassDirection)


    if -tolerance <= angleDiff <= tolerance:
        return "done"
    else:
        if radius < WHEELBASE:
            radius = WHEELBASE

        ang = math.degrees(math.asin(WHEELBASE/radius))
        if direction == "left":
            servo.write(ang)
            return "turn left"
        elif direction == "right":
            servo.write(-ang)
            return "turn right"


def getAngularDiff(intendedAngle, currentAng): # cw => -ve ccw => +ve 

    angDiff = intendedAngle - currentAng

    if angDiff > 180:
        angDiff -= 360
    elif angDiff < -180:
        angDiff += 360
    return -angDiff

prevError1 = 0
def keepStraight(headingDirection):

    global compassDirection
    global prevError1

    if headingDirection < 0:
        headingDirection += 360
    elif headingDirection > 360:
        headingDirection -= 360

    error = getAngularDiff(headingDirection, compassDirection) * (90/180)

    Kp, Kd, Ki =  1, 0, 0
    
    P = error   
    D = error - prevError1

    prevError1 = error

    return P*Kp + D*Kd

prevError2 = 0
def keepInMiddle(headingDirection, leftDist, rightDist, setDist):

    global compassDirection
    global prevError2

    ang = math.radians(getAngularDiff(compassDirection, headingDirection))
    error = round(leftDist*math.cos(ang) - rightDist*math.cos(ang)) + setDist

    Kp, Kd, Ki = 1, 0, 0

    P = error 
    D = error - prevError2

    prevError2 = error

    return P*Kp + D*Kd

def sideTrack(fixedDist, distance):

    Kp = 1
    P = fixedDist - distance

    return (P * Kp)

def scale(var, min1, max1, min2, max2):
    return (var/(max1-min1)) * (max2-min2) 

circularDirection = "CW"

def forward():
    car.speed(20)

def stop(): 
    car.speed(0)
    servo.write(0)

def center():
    servo.write(0)

def right():
    servo.write(45)

def left():
    servo.write(-45)


# Map keys to actions
key_action_map = {
    'w': forward,
    'a': left,
    's': stop,
    'd': right
}

# Function to handle key press events
def on_press(key):
    try:
        if key.char in key_action_map:
            key_action_map[key.char]()
    except AttributeError:
        # Handle special keys here if needed
        pass

# Function to handle key release events (e.g., stop movement)
def on_release(key):
    if key.char in key_action_map:
        center()
    if key == keyboard.Key.esc:
        # Stop listener on 'esc' key press
        return False


def main():

    # Start listening to the keyboard events
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()
            

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        GPIO.cleanup()


