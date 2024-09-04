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

# modules for camera vision 
import cv2
from PIL import Image      
from picamera2 import Picamera2
import modules.TrafficSignDetection as Sign

GPIO.setwarnings(False) # turn off warnings for pins (if pins were previously used and not released properly there will be warnings)
GPIO.setmode(GPIO.BOARD) # pin name convention used is pin numbers on board
factory = PiGPIOFactory()

#variables
WHEELBASE = 12          # vehicle wheelbase in cm
TOTALROUNDS = 3
compassDirection = 0    # headings from the compass module
drivingDirection = "CW" # round driving direction


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

def getAngularDiff(intendedAngle, currentAng): # cw => -ve ccw => +ve 

    angDiff = intendedAngle - currentAng

    if angDiff > 180:
        angDiff -= 360
    elif angDiff < -180:
        angDiff += 360
    return -angDiff

# radius is in cm
def turn(radius, headingDirection, direction, curAct, nextAct): 

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
            return curAct
        elif direction == "right":
            servo.write(-ang)
            return curAct

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
    ignore = False
    start = False
    noOfCornerTurns = 0
    headingDirection = 0
    turnReduction = 0

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

        compass.setHome(GPIO.input(startBut1)) # set direction value to 0 when button pressed

        capture = picam2.capture_array()

        if capture is None:
            print("Failed to capture frame")
            continue

        frame = capture
        frame = cv2.flip(frame, 0) # Flip vertically
        frame = cv2.flip(frame, 1) # Flip horizontally

        # Create TrafficSign objects
        green = Sign.TrafficSign(frame, (0, 255, 0), (40, 100, 100), (70, 255, 255), 8)
        red = Sign.TrafficSign(frame, (0, 0, 255), (0, 255, 27/255), (0, 69/255, 255), 8)

        # Draw bounding box for the largest detected object
        frame, greenSign = green.getBoundingBox()
        frame, redSign = red.getBoundingBox()

        # Draw center line and distances
        cv2.line(frame, (frame.shape[1]//2, 0), (frame.shape[1]//2, frame.shape[0]), (0, 255, 255), thickness = 3) # Centre line

        # Optionally, display information about the largest detected objects
        if greenSign:
            cv2.putText(frame, f"Green object: {greenSign[2]*greenSign[3]} px", (frame.shape[1] - 350, 40), cv2.FONT_HERSHEY_TRIPLEX, 1.0, (0, 255, 0), 1)

        if redSign:
            cv2.putText(frame, f"Red object: {redSign[2]*redSign[3]} px", (20, 40), cv2.FONT_HERSHEY_TRIPLEX, 1.0, (0, 0, 255), 1)

        cv2.imshow('Camera', frame)  # Show video


        #print(f"Front: {frontDist} But1: {GPIO.input(startBut1)}  But2: {GPIO.input(startBut2)} compass: {compassDirection} headingDirection: {headingDirection}")

        if GPIO.input(startBut1) == 9: # start the program
            start = True

            # reset variables
            headingDirection = 0
            canTurn = True
            noOfCornerTurns = 0
            action = None
            ignore = False

            LED.rgb(255,255,255)
            time.sleep(0.5)
        
        if start:
            
            if ignore:
                print("Ignore")
            elif redSign:
                action = "avoid right"
                headingDirection += 90

            elif greenSign:
                action = "avoid left"
                headingDirection -= 90

            elif frontDist <= 70 and canTurn:

                canTurn = False

                if noOfCornerTurns == 1: # determine driving direction
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

                noOfCornerTurns += 1
                LED.rgb(0,0,255) # LED blue


            if frontDist >= 130: # reset variable
                canTurn = True

            if noOfCornerTurns == 4*TOTALROUNDS + turnReduction and frontDist >= 150 and frontDist <= 160: # stop when finished
                start = False



            match action:

                case "corner right":
                    action = turn(20, headingDirection, "right", nextAct=None)

                case "corner left":
                    action = turn(20, headingDirection, "left", nextAct=None)

                case "avoid right":
                    action = turn(15, headingDirection, "right", nextAct="recover left")

                    if action == "recover left":
                        headingDirection -= 90
                        ignore = True
                
                case "avoid left":
                    action = turn(15, headingDirection, "right", nextAct="recover right")

                    if action == "recover right":
                        headingDirection += 90
                        ignore = True

                case "recover right":
                    action = turn(15, headingDirection, "right", nextAct=None)

                    if action == None:
                        
                        ignore = False
                
                case "recover left":
                    action = turn(15, headingDirection, "right", nextAct=None)
                    
                    if action == None:
                        if noOfCornerTurns == 4*3 - 1: # U turn if last sign is red
                            action = "u turn"
                            headingDirection += 180
                            ignore = True

                            turnReduction = -2

                            if drivingDirection == "ACW":
                                drivingDirection = "CW"
                            else:
                                drivingDirection = "ACW"                               
                        else:
                            ignore = False

                case "u turn":
                    action = turn(20, headingDirection, "right", nextAct=None)

                    if action == None:
                        ignore = False
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

        if cv2.waitKey(1) & 0xFF == ord('q'):  # Break out of loop if 'q' is pressed
            break

    cv2.destroyAllWindows()
            

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        GPIO.cleanup()
        cv2.destroyAllWindows()


