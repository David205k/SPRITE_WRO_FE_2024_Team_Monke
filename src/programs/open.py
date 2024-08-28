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

#variables
WHEELBASE = 12          # vehicle wheelbase in cm
TOTALROUNDS = 3
compassDirection = 0    # headings from the compass module
drivingDirection = "CW" # round driving direction

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

servo = myservo.myServo(gpioPin=5, startPos=0, offset=-7, minAng=-70, maxAng=70)
car = Tb6612fng.motor(stby=37, pwmA=35, ai1=36, ai2=40) 
LED = RGB.LED(red=8, blue=12, green=10)  

us2 = DistanceSensor(echo=27, trigger=22, max_distance=3, pin_factory=factory) # pins are gpio pins
us3 = DistanceSensor(echo=10, trigger=9, max_distance=3, pin_factory=factory) # pins are gpio pins
us4 = DistanceSensor(echo=6, trigger=13, max_distance=3, pin_factory=factory) # pins are gpio pins

startBut1 = 16
startBut2 = 18
GPIO.setup(startBut1,GPIO.IN)
GPIO.setup(startBut2,GPIO.IN)

def getAngularDiff(intendedAngle, currentAng): # cw => -ve ccw => +ve 

    angDiff = intendedAngle - currentAng

    if angDiff > 180:
        angDiff -= 360
    elif angDiff < -180:
        angDiff += 360
    return -angDiff

# radius is in cm
def turn(radius, headingDirection, direction, nextAct): 

    angTolerance = 5

    angleDiff = getAngularDiff(headingDirection, compassDirection)

    if -angTolerance <= angleDiff <= angTolerance:
        return nextAct
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

def keepStraight(headingDirection):

    global compassDirection

    error = getAngularDiff(headingDirection, compassDirection)

    Kp =  0.5
    P = error   

    return P*Kp

def keepInMiddle(headingDirection, leftDist, rightDist, setDist):

    global compassDirection

    ang = math.radians(getAngularDiff(compassDirection, headingDirection))
    error = round(leftDist*math.cos(ang) - rightDist*math.cos(ang)) + setDist

    Kp = 1

    P = error 

    return P*Kp 

def main():

    global drivingDirection
    global compassDirection
    global TOTALROUNDS

    action = None
    canTurn = True
    start = False
    noOfTurns = 0
    headingDirection = 0

    while True:

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

        if headingDirection < 0:
            headingDirection += 360
        elif headingDirection > 360:
            headingDirection -= 360

        compass.calibrate(GPIO.input(startBut1)) # set direction value to 0 when button pressed

        #print(f"Front: {frontDist} But1: {GPIO.input(startBut1)}  But2: {GPIO.input(startBut2)} compass: {compassDirection} headingDirection: {headingDirection}")

        if GPIO.input(startBut1): # start the program
            start = True

            # reset variables
            headingDirection = 0
            canTurn = True
            noOfTurns = 0
            action = None

            LED.rgb(255,255,255)
            time.sleep(0.5)
        
        if start:

            if frontDist <= 70 and canTurn:

                canTurn = False

                if noOfTurns == 1: # determine driving direction
                    if leftDist <= rightDist:
                        drivingDirection = "CW"
                    elif leftDist > rightDist:
                        drivingDirection = "ACW"
                
                if drivingDirection == "CW":
                    action = "corner right"
                    headingDirection += 90
                else:
                    action = "corner left"
                    headingDirection -= 90

                noOfTurns += 1
                LED.rgb(0,0,255) # LED blue

            if frontDist >= 200: # reset variable
                canTurn = True

            if noOfTurns == 4*TOTALROUNDS and frontDist >= 150 and frontDist <= 160: # stop when finished
                start = False


            match action:

                case "corner right":
                    action = turn(20, headingDirection, "right", nextAct=None)

                case "corner left":
                    action = turn(20, headingDirection, "left", nextAct=None)

                case _:  

                    LED.rgb(0,255,0)
                    angle = keepStraight(headingDirection)
                    servo.write(angle)
                    car.speed(30)

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


