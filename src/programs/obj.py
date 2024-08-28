import RPi.GPIO as GPIO # use RPi library for controlling GPIO pins
import time
import math
from gpiozero.pins.pigpio import PiGPIOFactory
   
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
TOTALROUNDS = 4
compassDirection = 0

# camera vision
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
def turn(radius, direction): 

    if radius < WHEELBASE:
        radius = WHEELBASE

    ang = math.degrees(math.asin(WHEELBASE/radius))

    if (direction == "left"):
        servo.write(ang)
    elif (direction == "right"):
        servo.write(-ang)

def getAngularDiff(intendedAngle, currentAng):

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

    Kp, Kd, Ki = 0.5, 0, 0

    P = error 
    D = error - prevError2

    prevError2 = error

    return P*Kp + D*Kd

def scale(var, min1, max1, min2, max2):
    return (var/(max1-min1)) * (max2-min2) 

def main():

    global compassDirection
    global TOTALROUNDS

    action = None
    canTurn = True
    start = False
    noOfTurns = 0

    headingDirection = 0

    while True:

        compass.calibrate(GPIO.input(startBut1) and GPIO.input(startBut2)) # set direction value to 0 when button pressed

        if GPIO.input(startBut1) and GPIO.input(startBut2): # start the car
            start = True

            LED.rgb(255,255,255)
            time.sleep(0.01)

            # reset variables
            headingDirection = 0
            canTurn = True
            noOfTurns = 0
        
        if start:

            # get US distances in cm
            frontDist, leftDist, rightDist = round(us4.distance*100), round(us2.distance*100), round(us3.distance*100)

            # get compass direction
            try:
                compassDirection = compass.getAngle()
            except OSError:
                while True:
                    print("Trying to connect to compass...")
                    try:
                        compassDirection = compass.getAngle()
                    except OSError:
                        continue 
                    print("Connection successful!")
                    break 

            turnDist = 70 # dist in cm
            if frontDist <= turnDist and canTurn:

                canTurn = False

                if leftDist <= rightDist:
                    turn(turnDist - 100/2, "right")
                    headingDirection += 90
                elif leftDist > rightDist:
                    turn(turnDist - 100/2, "left")
                    headingDirection -= 90

                noOfTurns += 1
                LED.rgb(0,0,255) # blue LED
                time.sleep(2)
            else:
                LED.rgb(0,255,0)
                angle = keepStraight(headingDirection)

                servo.write(angle)

                car.speed(30)

                print(f"Heading: {compass.heading} .Front: {frontDist} Left: {leftDist} right: {rightDist} direction: {compassDirection} angle: {angle}")

            if frontDist >= turnDist+100: # reset variable
                canTurn = True

            if frontDist <= 150:
                start = False
        else:
            # reset vehicle components
            servo.write(0)
            car.speed(0)
            LED.off()
            

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        GPIO.cleanup()


