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

# modules for camera vision
import cv2
from picamera2 import Picamera2
import numpy as np
import modules.TrafficSignDetection as Sign

GPIO.setwarnings(False) # turn off warnings for pins (if pins were previously used and not released properly there will be warnings)
GPIO.setmode(GPIO.BOARD) # pin name convention used is pin numbers on board
factory = PiGPIOFactory()

#variables
WHEELBASE = 12          # vehicle wheelbase in cm
TOTALROUNDS = 3         # total number of rounds to be completed  

drivingDirection = "CW"     # round driving direction
headingDirection = 0        # direction in which the robot should be heading in
noOfTurns = 0
noOfSigns = 0
noOfSignsEncountered = 0

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

servo = myservo.myServo(gpioPin=5, startPos=0, offset=-4, minAng=-70, maxAng=70)
car = Tb6612fng.motor(stby=37, pwmA=35, ai1=36, ai2=40) 
LED = RGB.LED(red=8, blue=12, green=10)  

us2 = DistanceSensor(echo=27, trigger=22, max_distance=3, pin_factory=factory) # pins are gpio pins
us3 = DistanceSensor(echo=10, trigger=9, max_distance=3, pin_factory=factory) # pins are gpio pins
us4 = DistanceSensor(echo=6, trigger=13, max_distance=3, pin_factory=factory) # pins are gpio pins

# two pins are connected to the start button as safeguard
startBut1, startBut2 = 16, 18
GPIO.setup(startBut1,GPIO.IN)
GPIO.setup(startBut2,GPIO.IN)

# camera vision 
# Define HSV color ranges for blue and orange
blue_lower, blue_upper = np.array([80, 130, 50]), np.array([140, 180, 90])
orange_lower, orange_upper = np.array([4, 100, 80]), np.array([20, 200, 150])

# traffic sign hsv values
green_lower, green_upper = (30, 100, 70), (100, 255, 200) 
red1_lower, red1_upper = (0, 210, 60), (4, 255, 255)
red2_lower, red2_upper = (170, 210, 60), (179, 255, 255)

picam2 = Picamera2()
picam2.preview_configuration.main.size=(1920,1000)
picam2.preview_configuration.main.format = 'RGB888'
picam2.start()

def getHsvAtMousePos(event, x, y, flags, param): # print pixel hsv at mouse location

    global hsv_frame

    if event == cv2.EVENT_MOUSEMOVE:  # If the mouse is moved
        # pass
        print(f"pixel hsv: {hsv_frame[y, x]}")

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

def getDrivingDirection(hsv_frame):

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
    
    # read compass angle (catch exception when compass is not connected proper ly)
    while True:
        try:
            compassDirection = compass.getAngle()
        except OSError:
            print("trying to connect to compass")
            continue
        break

doOnce = 0
doOnce2 = 0
start_time = 0
def executeAction(action):

    global headingDirection, drivingDirection
    global doOnce, doOnce2, start_time

    completed = None

    match action[0].split()[0]:

        case "corner_turn": # corner turn
            LED.rgb(0,0,255) # blue light

            turnRadius = int(action[0].split()[1])
            direction = "right" if drivingDirection == "CW" else "left"

            completed = turn(turnRadius, direction, headingDirection, tolerance=10)

            car.speed(30)

        # case "avoid_right":
    
        #     LED.rgb(255,0,255) # purple light

        #     completed = turn(20, "right", heading=int(action[0].split()[1]), tolerance=10)

        # case "recover_left":

        #     LED.rgb(255,255,0) # yellow light

        #     completed = turn(20, "left", heading=headingDirection, tolerance=5)
        #     car.speed(20)
            
        # case "avoid_left":
    
        #     LED.rgb(255,0,255) # purple light

        #     completed = turn(20, "left", heading=int(action[0].split()[1]), tolerance=10)

        # case "recover_right":

        #     LED.rgb(255,255,0) # yellow light

        #     completed = turn(20, "right", heading=headingDirection, tolerance=10)
        #     car.speed(20)

        case "avoid_obs_right": # red

            completed = turn(15, "right", heading=int(action[0].split()[1]), tolerance=10)
            car.speed(20)

        case "recover_obs_left": # red

            completed = turn(15, "left", heading=headingDirection, tolerance=10)
            car.speed(15)

        case "avoid_obs_left": # green

            completed = turn(15, "left", heading=int(action[0].split()[1]), tolerance=10)
            car.speed(20)

        case "recover_obs_right": # green

            LED.rgb(0, 100, 0) 

            completed = turn(15, "right", heading=headingDirection, tolerance=10)
            car.speed(15)

        case "u_turn": # for last red sign

            if doOnce2 == 0:
                doOnce2 = 1
                headingDirection += 180

            turnRadius = int(action[0].split()[1])

            completed = turn(turnRadius, "left", heading=headingDirection, tolerance=5)
            car.speed(20)

        case "travel": # drive forward for certain amt of time

            delay = float(action[0].split()[1])

            if doOnce == 0:
                doOnce = 1
                start_time = time.time()

            if time.time() - start_time >= delay or frontDist <= 10:
                completed = True
                doOnce = 0
            else: 
                completed = False

            car.speed(20)   
            servo.write(0)
 
        case "stop": #stop for certain amount of time

            delay = float(action[0].split()[1])

            if doOnce == 0:
                doOnce = 1
                start_time = time.time()

            if time.time() - start_time >= delay:
                completed = True
                doOnce = 0
            else: 
                completed = False

            car.speed(0)   
            servo.write(0)

        case "run":

            LED.rgb(0,255,0) # green light
            car.speed(20)
            servoAng = keepStraight()
            servo.write(servoAng)
            completed = True

    if completed == True:
        action.popleft()
            
hsv_frame = None

def main():

    actionQueue = deque()   
    start = False
    canTurn = False
    startFront = startLeft = startRight = 0

    global frontDist, leftDist, rightDist 
    global actualLeft, actualRight
    global headingDirection, compassDirection, noOfTurns, drivingDirection
    global hsv_frame

    while True:

        frame = picam2.capture_array()
        frame = cv2.flip(frame, 0) # Flip vertically
        frame = cv2.flip(frame, 1) # Flip horizontally
        
        # resize frame
        width = int(frame.shape[1] * 0.1)
        height = int(frame.shape[0] * 0.1)
        # frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)

        # Convert the frame to HSV color space
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        redMask = cv2.inRange(hsv_frame, red1_lower, red1_upper)  # Create mask for the color
        greenMask = cv2.inRange(hsv_frame, green_lower, green_upper)  # Create mask for the color

        green = Sign.TrafficSign(frame, boxColor=(0, 255, 0), lower=green_lower, upper=green_upper, minWidth=200, minHeight=300)
        red = Sign.TrafficSign(frame, boxColor=(0, 0, 255), lower=red1_lower, upper=red1_upper, lower2=red2_lower, upper2=red2_upper, minWidth=200, minHeight=300)

        # Draw bounding box for the largest detected object
        frame, greenSign = green.printLargestBbox()
        frame, redSign = red.printLargestBbox()

        # print center line and dist
        cv2.line(frame, (frame.shape[1]//2, 0), (frame.shape[1]//2, frame.shape[0]), (0, 255, 255), thickness = 3) #centre line
        cv2.putText(frame, "Dist: " + str(green.dist), (frame.shape[1] - 120, 440), cv2.FONT_HERSHEY_TRIPLEX, 1.0, (0, 255, 0), 1)
        cv2.putText(frame, "Dist: " + str(red.dist), (20, 440), cv2.FONT_HERSHEY_TRIPLEX, 1.0, (0, 0, 255), 1)

        # cv2.imshow('camera', frame)
        # cv2.imshow('red mask', redMask)
        # cv2.imshow('green mask', greenMask)

        # cv2.setMouseCallback('camera', getHsvAtMousePos)

        readSensors()

        # print(f"clrDetect: {red.detectClr()} actionQueue: {actionQueue} compassDirec: {compassDirection}, heading: {headingDirection} turns: {noOfTurns} drivingDirection: {drivingDirection} f: {frontDist}, l: {leftDist} r: {rightDist}")

        # start the robot when button pressed
        if GPIO.input(startBut1) and GPIO.input(startBut2):
            start = True

            # reset variabls
            headingDirection = noOfTurns = noOfSignsEncountered = 0
            canTurn = True

            noOfSigns = 0

            startFront = frontDist
            startLeft = leftDist
            startRight  = rightDist

            actionQueue.clear()
            compass.setHome()

            readSensors()   

            drivingDirection = getDrivingDirection(hsv_frame)
        
        # stop the robot when 3 rounds completed
        if noOfTurns == 4 * TOTALROUNDS and frontDist >= 150 and frontDist <=160 and canTurn == True:
            start = False
            print("Driving completed")

        if start:

            if actionQueue:
                pass
            elif frontDist <= 70 and canTurn and red.detectClr() == None and green.detectClr() == None and ((drivingDirection == "CW" and rightDist >= 20) or (drivingDirection == "ACW" and leftDist >= 20)) :

                canTurn = False

                car.speed(30)

                if noOfTurns == 0 and drivingDirection == "unknown":
                    if leftDist <= rightDist:
                        drivingDirection = "CW"
                    elif leftDist > rightDist:
                        drivingDirection = "ACW"

                headingDirection += 90 if drivingDirection == "CW" else -90 

                noOfTurns += 1

                actionQueue = deque([f"corner_turn {20}", "run"])

            # elif leftDist <= 1:

            #     actionQueue = deque([f"avoid_right {headingDirection+30}", "recover_left"])

            # elif rightDist <= 1:

            #     actionQueue = deque([f"avoid_left {headingDirection-30}", "recover_right"])

            elif redSign:

                LED.rgb(100, 0, 0)

                # get travel time
                if red.x > frame.shape[1]//2 + 10: # if red is on the right of robot
                    travelTime = ((red.x - frame.shape[1]//2) / 1920) * 3.5#(7 * (50/100))
                else:
                    travelTime = 0

                print(f"x: {red.x} travel error: {(red.x - frame.shape[1]//2)}  time: {travelTime} ")

                if noOfSignsEncountered == noOfSigns*TOTALROUNDS and noOfTurns > 8:
                    actionQueue = deque([f"avoid_obs_right {headingDirection+70}", f"travel {travelTime}", "recover_obs_left", "u_turn 30"])#, f"travel {0.4}", f"avoid_left {headingDirection-45}", "recover_right"])
                else:
                    actionQueue = deque([f"avoid_obs_right {headingDirection+70}", f"travel {travelTime}", "recover_obs_left", f"travel {7 * 22/100}", f"avoid_obs_left {headingDirection-70}", "recover_obs_right", f"stop {0.5}"])#, f"travel {0.4}", f"avoid_left {headingDirection-45}", "recover_right"])

                if noOfTurns < 5:
                    noOfSigns += 1

                noOfSignsEncountered += 1

            elif greenSign:

                LED.rgb(0, 100, 0)

                if green.x < frame.shape[1]//2 - 10:  # if green is on the left of robot
                    travelTime = ((frame.shape[1]//2-green.x)/ 1920) * 3.5 #(7 * (50/100))
                else:
                    travelTime = 0

                print(f"x: {green.x} travel error: {(frame.shape[1]//2)-green.x / 1920}  time: {travelTime} ")
                
                actionQueue = deque([f"avoid_obs_left {headingDirection-70}", f"travel {travelTime}", "recover_obs_right", f"travel {7 * 22/100}", f"avoid_obs_right {headingDirection+90}", "recover_obs_left", f"stop {0.5}"])#, f"travel {0.4}", f"avoid_left {headingDirection-45}", "recover_right"])

                if noOfTurns < 5:
                    noOfSigns += 1

                noOfSignsEncountered += 1

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


