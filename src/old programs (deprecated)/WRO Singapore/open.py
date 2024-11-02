import RPi.GPIO as GPIO # use RPi library for controlling GPIO pins
import math
import time
from gpiozero.pins.pigpio import PiGPIOFactory
from collections import deque

# modules for controlling components
import modules.Tb6612fngControl as Tb6612fng
import modules.RGBLEDControl as RGB
import modules.HMC5883LControl as HMC5883L
import modules.ServoControl_gpiozero as myservo
from gpiozero import DistanceSensor

import cv2
from picamera2 import Picamera2
import numpy as np

GPIO.setwarnings(False) # turn off warnings for pins (if pins were previously used and not released properly there will be warnings)
GPIO.setmode(GPIO.BOARD) # pin name convention used is pin numbers on board
factory = PiGPIOFactory()

#variables
WHEELBASE = 12          # vehicle wheelbase in cm
TOTALROUNDS = 3         # total number of rounds to be completed  

drivingDirection = "CW"     # round driving direction
headingDirection = 0        # direction in which the robot should be heading in
noOfTurns = 0

compassDirection = 0  
leftDist = rightDist = frontDist = 0
actualLeft = actualRight = 0


# initialise components  
while True:
    try:
        compass = HMC5883L.compass(addr=0x1E)
    except OSError:
        print("Trying to connect to compass...")
        continue 
    print("Connection to compass successful!")
    break 

servo = myservo.myServo(gpioPin=5, startPos=0, offset=-5, minAng=-70, maxAng=70)
car = Tb6612fng.motor(stby=37, pwmA=35, ai1=36, ai2=40) 
LED = RGB.LED(red=8, blue=12, green=10)  

us2 = DistanceSensor(echo=27, trigger=22, max_distance=3, pin_factory=factory) # pins are gpio pins
# us3 = DistanceSensor(echo=10, trigger=9, max_distance=3, pin_factory=factory) # pins are gpio pins
us3 = DistanceSensor(echo=7, trigger=8, max_distance=3, pin_factory=factory) # pins are gpio pins
us4 = DistanceSensor(echo=6, trigger=13, max_distance=3, pin_factory=factory) # pins are gpio pins

# two pins are connected to the start button as safeguard
startBut1 = 16
startBut2 = 18
GPIO.setup(startBut1,GPIO.IN)
GPIO.setup(startBut2,GPIO.IN)

# camera vision 
# Define HSV color ranges for blue and orange
blue_lower, blue_upper = np.array([100, 20, 50]), np.array([150, 110, 90])
orange_lower, orange_upper = np.array([0, 145, 80]), np.array([10, 255, 150])

picam2 = Picamera2()
picam2.preview_configuration.main.size=(1920,1000)
picam2.preview_configuration.main.format = 'RGB888'
picam2.start()

def getAngularDiff(intendedAngle, currentAng): # cw => -ve ccw => +ve 

    angDiff = intendedAngle - currentAng

    if angDiff > 180:
        angDiff -= 360
    elif angDiff < -180:
        angDiff += 360
    return -angDiff

def turn(radius, direction, heading, tolerance = 10):  # turn radius in cm

    global compassDirection

    angleDiff = getAngularDiff(heading, compassDirection)

    if -tolerance <= angleDiff <= tolerance:
        return True
    else:
        if radius < WHEELBASE: # account for radius smaller than wheel base
            radius = WHEELBASE

        ang = math.degrees(math.asin(WHEELBASE/radius))

        if direction == "left":
            servo.write(ang)
        elif direction == "right":
            servo.write(-ang)

        return False

def keepStraight():

    global compassDirection, headingDirection

    error = getAngularDiff(headingDirection, compassDirection)

    Kp =  1
    P = error   

    return P*Kp

def keepInMiddle(leftDist, rightDist, setDist):

    global compassDirection, headingDirection

    ang = math.radians(getAngularDiff(compassDirection, headingDirection))
    error = round(leftDist*math.cos(ang) - rightDist*math.cos(ang)) + setDist

    Kp = 1

    P = error 

    return P*Kp 

def readSensors():

    global frontDist, leftDist, rightDist 
    global actualLeft, actualRight
    global compassDirection, headingDirection

    if headingDirection < 0:
        headingDirection += 360
    elif headingDirection > 360:
        headingDirection -= 360

    frontDist, leftDist, rightDist = round(us4.distance*100), round(us2.distance*100), round(us3.distance*100)
    
    ang = math.radians(getAngularDiff(compassDirection, headingDirection))
    actualLeft = abs(leftDist*math.cos(ang))
    actualRight = abs(rightDist*math.cos(ang))
    
    # read compass angle (catch exception when compass is not connected properly)
    while True:
        try:
            compassDirection = compass.getAngle()
        except OSError:
            print("trying to connect to compass")
            continue
        break

def getRoundDirection(hsv_frame, frame):
    BlueAngleSum = 0
    OranAngleSum = 0
    BlueMean = 0
    OranMean = 0
    for d in range(0,5):
        # Create masks for blue and orange
        blue_mask = cv2.inRange(hsv_frame, blue_lower, blue_upper)
        orange_mask = cv2.inRange(hsv_frame, orange_lower, orange_upper)
        # Use Hough Line Transform to detect lines
        lineB = cv2.HoughLinesP(blue_mask, 1, np.pi / 180, threshold=100, minLineLength=100, maxLineGap=5)
        lineO = cv2.HoughLinesP(orange_mask, 1, np.pi / 180, threshold=100, minLineLength=100, maxLineGap=5)
        
        # Check if any lines were found
        if lineB is not None:
            for line in lineB:
                xB1, yB1, xB2, yB2 = line[0]
        
            # Calculate the angle of the line
            angleB = abs(np.degrees(np.arctan2(yB2 - yB1, xB2 - xB1)))
            BlueAngleSum += angleB
            print(f" Blue Line from ({xB1}, {yB1}) to ({xB2}, {yB2}) has an angle of {angleB:.2f} degrees")
    
            # Draw the line on the image
            cv2.line(frame, (xB1, yB1), (xB2, yB2), (0, 255, 0), 2)
        if lineO is not None:
            for line in lineO:
                xO1, yO1, xO2, yO2 = line[0]
        
            # Calculate the angle of the line
            angleO = abs(np.degrees(np.arctan2(yO2 - yO1, xO2 - xO1)))
            OranAngleSum += angleO
            print(f"Orange Line from ({xO1}, {yO1}) to ({xO2}, {yO2}) has an angle of {angleO:.2f} degrees")
    
            # Draw the line on the image
            cv2.line(frame, (xO1, yO1), (xO2, yO2), (0, 255, 0), 2)
        else:
            return "unknown"
    BlueMean = abs(BlueAngleSum)/5
    OranMean = abs(OranAngleSum)/5
    if BlueMean > OranMean:
        LED.rgb(255,165,0)
        return "CW"
    if OranMean > BlueMean:
        LED.rgb(0,0,225)
        return "ACW"
    else:   
        print("No blue or orange detected in the frame")
        return "unknown"
    
doOnce = 0
start_time = 0
def executeAction(action):

    global headingDirection, drivingDirection
    global doOnce, start_time

    TurnHeadingDirection = 0

    completed = None

    match action[0].split()[0]:

        case "corner_turn":
            LED.rgb(0,0,255) # blue light

            direction = "right" if drivingDirection == "CW" else "left"
            if drivingDirection == "CW":
                TurnHeadingDirection = headingDirection - 10
            else:
                TurnHeadingDirection = headingDirection + 10
            completed = turn(20, direction, TurnHeadingDirection, tolerance=15)

            car.speed(50)

        case "avoid_right":
    
            LED.rgb(255,0,255) # purple light

            completed = turn(25, "right", heading=int(action[0].split()[1]), tolerance=10)

            car.speed(40)

        case "recover_left":

            LED.rgb(255,255,0) # yellow light

            completed = turn(25, "left", heading=headingDirection+5, tolerance=5)
            car.speed(40)
            
        case "avoid_left":
    
            LED.rgb(255,0,255) # purple light

            completed = turn(25, "left", heading=int(action[0].split()[1]), tolerance=10)

            car.speed(40)

        case "recover_right":

            LED.rgb(255,255,0) # yellow light

            completed = turn(25, "right", heading=headingDirection-5, tolerance=5)
            car.speed(40)
            
        case "reverse": # drive forward for certain amt of time

            delay = float(action[0].split()[1])

            if doOnce == 0:
                doOnce = 1
                start_time = time.time()

            if time.time() - start_time >= delay:
                completed = True
                doOnce = 0
            else: 
                completed = False

            car.speed(-20)   
            servo.write(0)
            
        case "run":
            LED.rgb(0,255,0) # green light
            car.speed(70)
            servoAng = keepStraight()
            servo.write(servoAng)
            completed = True

    if completed == True:
        action.popleft()
            

def main():

    actionQueue = deque()   
    start = False
    canTurn = False

    startFront = 0

    global frontDist, leftDist, rightDist 
    global actualLeft, actualRight
    global headingDirection, compassDirection, noOfTurns, drivingDirection

    while True:

        frame = picam2.capture_array()
        frame = cv2.flip(frame, 0) # Flip vertically
        frame = cv2.flip(frame, 1) # Flip horizontally

        # Convert the frame to HSV color space
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        blue_mask = cv2.inRange(hsv_frame, blue_lower, blue_upper)
        orange_mask = cv2.inRange(hsv_frame, orange_lower, orange_upper)

        cv2.imshow('preview', frame)

        readSensors()

        print(f"compassDirec: {compassDirection}, heading: {headingDirection} drivingDirection: {drivingDirection} f: {frontDist}, l: {leftDist} r: {rightDist}")

        # start the robot when button pressed
        if GPIO.input(startBut1) and GPIO.input(startBut2):
            start = True

            # reset variabls
            headingDirection = noOfTurns = 0
            startFront = frontDist
            canTurn = True
            actionQueue.clear()
            compass.set_home()

            readSensors()   

            drivingDirection = getRoundDirection(hsv_frame, frame)
            time.sleep(0.5)

        
        # stop the robot when 3 rounds completed
        if noOfTurns == 4 * TOTALROUNDS and startFront - 25 <= frontDist <= startFront + 5 and canTurn == True:
            start = False

        if start:

            if actionQueue:
                pass
            elif frontDist <= 80 and canTurn: 

                canTurn = False

                if noOfTurns == 0 and drivingDirection == "unknown":
                    if leftDist <= rightDist:
                        drivingDirection = "CW"
                    elif leftDist > rightDist:
                        drivingDirection = "ACW"

                headingDirection += 90 if drivingDirection == "CW" else -90 

                noOfTurns += 1

                actionQueue = deque(["corner_turn", "run"])

            elif leftDist <= 20:

                actionQueue = deque([f"avoid_right {headingDirection+45}", "recover_left"])

            elif rightDist <= 20:

                actionQueue = deque([f"avoid_left {headingDirection-45}", "recover_right"])

            else:
                actionQueue = deque(["run"])

            if frontDist >= 130:
                canTurn = True
            

            executeAction(actionQueue)
            
        else:
            car.speed(0)
            servo.write(0)
            LED.off()

        if cv2.waitKey(1) == ord('q'):
            break

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        picam2.stop()
        cv2.destroyAllWindows() 
        GPIO.cleanup()


