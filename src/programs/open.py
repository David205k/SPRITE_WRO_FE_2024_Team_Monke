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

servo = myservo.myServo(gpioPin=5, startPos=0, offset=-13, minAng=-70, maxAng=70)
car = Tb6612fng.motor(stby=37, pwmA=35, ai1=36, ai2=40) 
LED = RGB.LED(red=8, blue=12, green=10)  

us2 = DistanceSensor(echo=27, trigger=22, max_distance=3, pin_factory=factory) # pins are gpio pins
us3 = DistanceSensor(echo=10, trigger=9, max_distance=3, pin_factory=factory) # pins are gpio pins
us4 = DistanceSensor(echo=6, trigger=13, max_distance=3, pin_factory=factory) # pins are gpio pins

# two pins are connected to the start button as safeguard
startBut1 = 16
startBut2 = 18
GPIO.setup(startBut1,GPIO.IN)
GPIO.setup(startBut2,GPIO.IN)

# camera vision 
# Define HSV color ranges for blue and orange
blue_lower, blue_upper = np.array([80, 130, 50]), np.array([140, 180, 90])
orange_lower, orange_upper = np.array([0, 190, 80]), np.array([20, 255, 150])

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

    Kp =  0.5
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

def getRoundDirection(hsv_frame):

    # Create masks for blue and orange
    blue_mask = cv2.inRange(hsv_frame, blue_lower, blue_upper)
    orange_mask = cv2.inRange(hsv_frame, orange_lower, orange_upper)

    # Scan the frame from bottom to top
    for y in range(hsv_frame.shape[0]-1, -1, -1):  # Loop over rows (height)
        # Get the row masks
        blue_row = blue_mask[y, :]
        orange_row = orange_mask[y, :]

        # Check if blue or orange is detected first
        if np.any(blue_row):  # Check if any blue pixel is present

            print("Round is anti clockwise")
            LED.rgb(0,0,255) # blue light
            time.sleep(0.5)
            return "ACW"

        elif np.any(orange_row):  # Check if any orange pixel is present
            print("Round is clockwise")
            LED.rgb(255,69,0) # orange light
            time.sleep(0.5)
            return "CW"
    else:
        print("No blue or orange detected in the frame")
        return "unknown"

def executeAction(action):

    global headingDirection, drivingDirection

    completed = None

    match action[0].split()[0]:

        case "corner_turn":
            LED.rgb(0,0,255) # blue light

            direction = "right" if drivingDirection == "CW" else "left"
            completed = turn(20, direction, headingDirection, tolerance=10)

        case "avoid_right":
    
            LED.rgb(255,0,255) # purple light

            completed = turn(20, "right", heading=int(action[0].split()[1]), tolerance=10)

        case "recover_left":

            LED.rgb(255,255,0) # yellow light

            completed = turn(20, "left", heading=headingDirection, tolerance=5)
            car.speed(30)
            
        case "avoid_left":
    
            LED.rgb(255,0,255) # purple light

            completed = turn(20, "left", heading=int(action[0].split()[1]), tolerance=10)

        case "recover_right":

            LED.rgb(255,255,0) # yellow light

            completed = turn(20, "right", heading=headingDirection, tolerance=5)
            car.speed(30)
            
        case "run":
            LED.rgb(0,255,0) # green light
            car.speed(40)
            servoAng = keepStraight()
            servo.write(servoAng)
            completed = True

    if completed == True:
        action.popleft()
            

def main():

    actionQueue = deque()   
    start = False
    canTurn = False
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
            canTurn = True
            actionQueue.clear()
            compass.setHome()

            readSensors()   

            drivingDirection = getRoundDirection(hsv_frame)
        
        # stop the robot when 3 rounds completed
        if noOfTurns == 4 * TOTALROUNDS and frontDist >= 150 and frontDist <=160 and canTurn == True:
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

                actionQueue = deque([f"avoid_right {headingDirection+30}", "recover_left"])

            elif rightDist <= 20:

                actionQueue = deque([f"avoid_left {headingDirection-30}", "recover_right"])

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




# import RPi.GPIO as GPIO # use RPi library for controlling GPIO pins
# import time
# import math
# from gpiozero.pins.pigpio import PiGPIOFactory

# # modules for controlling components
# import modules.Tb6612fngControl as Tb6612fng
# import modules.RGBLEDControl as RGB
# import modules.HMC5883LControl as HMC5883L
# import modules.ServoControl_gpiozero as myservo
# from gpiozero import DistanceSensor

# GPIO.setwarnings(False) # turn off warnings for pins (if pins were previously used and not released properly there will be warnings)
# GPIO.setmode(GPIO.BOARD) # pin name convention used is pin numbers on board
# factory = PiGPIOFactory()

# #variables
# WHEELBASE = 12          # vehicle wheelbase in cm
# TOTALROUNDS = 3
# compassDirection = 0    # headings from the compass module
# drivingDirection = "CW" # round driving direction

# # initialise components  
# try: 
#     compass = HMC5883L.compass(addr=0x1E)
# except OSError:

#     while True:

#         print("Trying to connect to compass...")
#         try:
#             compass = HMC5883L.compass(addr=0x1E)
#         except OSError:
#             continue 
#         print("Connection successful!")
#         break 

# servo = myservo.myServo(gpioPin=5, startPos=0, offset=-13, minAng=-70, maxAng=70)
# car = Tb6612fng.motor(stby=37, pwmA=35, ai1=36, ai2=40) 
# LED = RGB.LED(red=8, blue=12, green=10)  

# us2 = DistanceSensor(echo=27, trigger=22, max_distance=3, pin_factory=factory) # pins are gpio pins
# us3 = DistanceSensor(echo=10, trigger=9, max_distance=3, pin_factory=factory) # pins are gpio pins
# us4 = DistanceSensor(echo=6, trigger=13, max_distance=3, pin_factory=factory) # pins are gpio pins

# startBut1 = 16
# startBut2 = 18
# GPIO.setup(startBut1,GPIO.IN)
# GPIO.setup(startBut2,GPIO.IN)

# def getAngularDiff(intendedAngle, currentAng): # cw => -ve ccw => +ve 

#     angDiff = intendedAngle - currentAng

#     if angDiff > 180:
#         angDiff -= 360
#     elif angDiff < -180:
#         angDiff += 360
#     return -angDiff

# # radius is in cm
# def turn(radius, headingDirection, direction, curAct, nextAct): 

#     angTolerance = 7

#     angleDiff = getAngularDiff(headingDirection, compassDirection)

#     if -angTolerance <= angleDiff <= angTolerance:
#         return nextAct
#     else:
#         if radius < WHEELBASE:
#             radius = WHEELBASE

#         ang = math.degrees(math.asin(WHEELBASE/radius))
#         if direction == "left":
#             servo.write(ang)
#             return curAct
#         elif direction == "right":
#             servo.write(-ang)
#             return curAct

# def keepStraight(headingDirection):

#     global compassDirection

#     error = getAngularDiff(headingDirection, compassDirection)

#     Kp =  0.5
#     P = error   

#     return P*Kp

# def keepInMiddle(headingDirection, leftDist, rightDist, setDist):

#     global compassDirection

#     ang = math.radians(getAngularDiff(compassDirection, headingDirection))
#     error = round(leftDist*math.cos(ang) - rightDist*math.cos(ang)) + setDist

#     Kp = 1

#     P = error 

#     return P*Kp 

# def main():

#     global drivingDirection
#     global compassDirection
#     global TOTALROUNDS

#     action = None
#     canTurn = True
#     start = False
#     noOfTurns = 0
#     headingDirection = 0
#     ignore = False
#     turnDist = 0

#     initialFront = initialLeft = initialRight = None

#     while True:

#         if headingDirection < 0:
#             headingDirection += 360
#         elif headingDirection > 360:
#             headingDirection -= 360

#         # get US distances in cm
#         frontDist, leftDist, rightDist = round(us4.distance*100), round(us2.distance*100), round(us3.distance*100)

#         #print(f"Front: {frontDist} But1: {GPIO.input(startBut1)}  But2: {GPIO.input(startBut2)} compass: {compassDirection} headingDirection: {headingDirection}")
#         if GPIO.input(startBut1) and GPIO.input(startBut2): # start the program
#             start = True

#             # reset variables
#             headingDirection = 0
#             canTurn = True
#             noOfTurns = 0
#             action = None
#             ignore = False

#             initialFront = frontDist
#             initialLeft = leftDist
#             initialRight = rightDist

#             LED.rgb(255,255,255)
#             time.sleep(0.5)
        
#             compass.calibrate(True) # set direction value to 0 when button pressed
#             doOnce = 0

#         if start:

#             # get compass direction
#             try:
#                 compassDirection = compass.getAngle()
#             except OSError:
#                 while True:
#                     print("Trying to connect to compass...")
#                     try:
#                         compassDirection = compass.getAngle()
#                     except OSError:
#                         continue 
#                     print("Connection successful!")
#                     break 

#             ang = math.radians(getAngularDiff(compassDirection, headingDirection))
#             actualLeft = abs(leftDist*math.cos(ang))
#             actualRight = abs(rightDist*math.cos(ang))

#             #print(f"actualLeft: {actualLeft}, compassDirection: {compassDirection}, headingDirection: {headingDirection}")
        
#             if frontDist <= 70 and canTurn:

#                 canTurn = False

#                 if noOfTurns == 0: # determine driving direction
#                     if leftDist <= rightDist:
#                         drivingDirection = "CW"
#                     elif leftDist > rightDist: 
#                         drivingDirection = "ACW"
                
#                 if drivingDirection == "CW":
#                     action = "corner right"
#                     headingDirection += 90
#                 else:
#                     action = "corner left"
#                     headingDirection -= 90

#                 noOfTurns += 1
#             # elif 0<= actualLeft <= 20 and leftDist <= 60 and frontDist >= 100:
#             #     print("turning back")
#             #     direc = headingDirection + 45
#             #     action = "avoid right"
#             #     ignore = True
#             #     turnDist = actualLeft
#             # elif 0<= actualRight <= 20 and rightDist <= 60 and frontDist >= 50 and action != "corner right" and action != "corner left":
#             #     print("turning back")
#             #     direc = headingDirection - 45
#             #     action = "avoid left"
#             #     ignore = True
#             #     turnDist = actualRight

#             if frontDist >= 130: # reset variable
#                 canTurn = True

#             if noOfTurns == 4*TOTALROUNDS and initialFront-10 <= frontDist <= initialFront+8: # stop when finished
#                 start = False

#             print(f"heading: {compass.heading} startPos: {compass.startPos} compass: {compassDirection} actualRight: {round(actualRight)} headingDirection: {headingDirection} front: {frontDist} left: {leftDist} right: {rightDist} angle:  noOfTurns: {noOfTurns} actualLeft: {actualLeft} action: {action}")

#             match action:

#                 case "corner right":
#                     action = turn(20, headingDirection, "right", curAct=action, nextAct=None)
#                     LED.rgb(0, 0, 255) # LED blue

#                 case "corner left":
#                     action = turn(20, headingDirection, "left", curAct=action, nextAct=None)
#                     LED.rgb(0, 0, 255) # LED blue

#                 case "avoid right":
#                     action = turn(15, direc, "right", curAct=action, nextAct="recover left")
#                     LED.rgb(255, 0, 255) # LED purple 
#                     car.speed(20)

#                     action = turn(15, headingDirection, "left", curAct=action, nextAct=None)
#                     action = turn(15, headingDirection, "left", curAct=action, nextAct=None)
#                     LED.rgb(255, 180, 155) # LED purple 
#                     car.speed(15)

#                     if action == None:
#                         ignore = False

#                 case "avoid left":
#                     action = turn(15, direc, "left", curAct=action, nextAct="recover right")
#                     LED.rgb(255, 0, 255) # LED purple 
#                     car.speed(20)

#                 case "recover right":
#                     action = turn(15, headingDirection, "right", curAct=action, nextAct=None)
#                     LED.rgb(255, 180, 155) # LED white
#                     car.speed(15)

#                     if action == None:
#                         ignore = False

#                 case _:  

#                     LED.rgb(0,255,0)
#                     angle = keepStraight(headingDirection)
#                     servo.write(angle)
#                     car.speed(40)
#         else:
#             # reset vehicle components
#             servo.write(0)
#             car.speed(0)
#             LED.off()

# if __name__ == "__main__":
#     try:
#         main()
#     except KeyboardInterrupt:
#         GPIO.cleanup()


