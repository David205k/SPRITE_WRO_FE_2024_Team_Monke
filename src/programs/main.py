import RPi.GPIO as GPIO  # use RPi library for controlling GPIO pins
import time
import math
import smbus
import subprocess

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

command = "sudo pigpiod"
process = subprocess.run(command, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

GPIO.setwarnings(False)  # turn off warnings for pins (if pins were previously used and not released properly there will be warnings)
GPIO.setmode(GPIO.BOARD)  # pin name convention used is pin numbers on board
factory = PiGPIOFactory()

# variables
WHEELBASE = 0.12
frontDist, leftDist, rightDist = 0, 0, 0
angle = 0
noOfRounds = 2

# camera vision
picam2 = Picamera2()
picam2.preview_configuration.main.size = (1920, 1000)
picam2.preview_configuration.main.format = 'RGB888'
picam2.start()

# initialize components
compass = HMC5883L.compass(addr=0x1E)
servo = AngularServo(5, min_angle=-90, max_angle=90, min_pulse_width=0.0004, max_pulse_width=0.0026, pin_factory=factory)
car = Tb6612fng.motor(stby=37, pwmA=35, ai1=36, ai2=40)
LED = RGB.LED(red=8, blue=12, green=10)

butPin = 16
GPIO.setup(butPin, GPIO.IN)

us2 = DistanceSensor(echo=27, trigger=22, max_distance=3)
us3 = DistanceSensor(echo=10, trigger=9, max_distance=3)
us4 = DistanceSensor(echo=6, trigger=13, max_distance=3)



def turn(radius, direction):
    angle = math.degrees(math.asin(WHEELBASE/radius))
    if direction == "left":
        servo.angle = angle - 7
    elif direction == "right":
        servo.angle = -angle - 7

def getAngularDiff(intendedAngle, currentAng):
    angDiff = intendedAngle - currentAng
    if angDiff > 180:
        angDiff -= 360
    elif angDiff < -180:
        angDiff += 360
    return -angDiff

def keepStraight(headingDirection):

    if headingDirection < 0:
        headingDirection += 360
    elif headingDirection > 360:
        headingDirection -= 360

    error = getAngularDiff(headingDirection, angle) * (90/180)
    Kp, Kd = 1, 0
    
    P = error

    PID = Kp*P - 7

    if PID > 90:
        PID = 90
    elif PID < -90:
        PID = -90

    servo.angle = PID

def cornerTurn(direction):
    LED.rgb(0, 0, 255)
    turn(0.15, direction)
    time.sleep(1)

def getDistOffset(headingDirection):
    ang = getAngularDiff(angle, headingDirection)
    return round(leftDist * math.cos(ang) - rightDist * math.cos(ang))

def keepInMiddle(headingDirection):
    error = getDistOffset(headingDirection)
    Kp, Kd, Ki = 1, 0, 0
    P = (error / (50 - 15)) * 90
    PID = P * Kp - 7

    if PID > 90:
        PID = 90
    elif PID < -90:
        PID = -90

    servo.angle = PID

done = 0
noOfTurns = 0

def main():
    action = 0
    headingDirection = 0

    global frontDist
    global leftDist
    global rightDist
    global angle
    global noOfTurns
    global done
    start = 0

    while True:
        compass.calibrate(GPIO.input(butPin))
        if GPIO.input(butPin) == 1:
            headingDirection = 0
            start = 1
            done = 0
            noOfTurns = 0
        
        if start == 1:
            # get US distances in cm
            frontDist, leftDist, rightDist = round(us4.distance * 100), round(us2.distance * 100), round(us3.distance * 100)

            try:
                angle = compass.getAngle()
            except OSError:

                while True:

                    print("Trying to connect to compass...")
                    try:
                        angle = compass.getAngle()
                    except OSError:
                        continue 
                    print("Connection successful!")
                    break 


            if frontDist <= 40 and done == 0 and noOfTurns < 4 * noOfRounds:
                noOfTurns += 1
                done = 1
                if leftDist <= rightDist:
                    headingDirection += 90
                    cornerTurn("right")
                elif leftDist > rightDist:
                    headingDirection -= 90
                    cornerTurn("left")
            else:
                LED.rgb(0, 255, 0)
                keepInMiddle(headingDirection)
                car.speed(30)

                if noOfTurns == 4 * noOfRounds and frontDist <= 118:
                    car.speed(0)
                    break

            if frontDist >= 50:
                done = 0

            print(f"distError: {getDistOffset(headingDirection)} heading: {headingDirection} left: {leftDist} right : {rightDist}")

        while noOfTurns == 4 * noOfRounds and GPIO.input(butPin) != 1:
            car.speed(0)
            LED.rgb(255, 255, 255)
            time.sleep(1)
            LED.off()
            time.sleep(1)

if __name__ == "__main__":
    main()
