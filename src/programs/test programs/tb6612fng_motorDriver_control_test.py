# example on using tb6612fng motor driver to control a DC Motor
# by David Lim 7/6/2024 Friday
import sys

sys.path.append('/home/monke/WRO FE 2024 (Repository)/src/programs/modules')

import RPi.GPIO as GPIO # use RPi library for controlling GPIO pins
import ServoControl_gpiozero as myservo
from gpiozero.pins.pigpio import PiGPIOFactory
import time

GPIO.setwarnings(False) # turn off warnings for pins (if 1if pins were previously used and not released properly there will be warnings)
GPIO.setmode(GPIO.BOARD) # pin name convention used is pin numbers on board

# motor driver connections
pwmA = 35
ai2 = 40
ai1 = 36
stby = 37

# set pins to output mode
GPIO.setup(pwmA,GPIO.OUT)
GPIO.setup(ai2,GPIO.OUT)
GPIO.setup(ai1,GPIO.OUT)
GPIO.setup(stby,GPIO.OUT)

# # set up pwm signal
pi_pwm = GPIO.PWM(pwmA, 100) # set pwm frequency to 100
pi_pwm.start(30) # set duty cycle to 30

# turn off pins at the start
GPIO.output(ai1, GPIO.LOW)
GPIO.output(ai2, GPIO.LOW)

GPIO.output(stby, GPIO.HIGH) # turn on motor driver

factory = PiGPIOFactory()
servo = myservo.myServo(gpioPin=5, startPos=20, offset=-13, minAng=-70, maxAng=70)


while True:

    userInput = "f20"
    direction, speed = userInput[0], userInput[1:]
    servo.write(0) 

    try:
        pi_pwm.start(int(speed))
    except ValueError:
        speed = 0

    if direction == 'f': # forward
        GPIO.output(ai1, GPIO.HIGH)
        GPIO.output(ai2, GPIO.LOW)

    elif direction == 'b': # forward
        GPIO.output(ai1, GPIO.LOW)
        GPIO.output(ai2, GPIO.HIGH)

    elif direction == 'r': # right
        servo.write(-45)


    elif direction == 'l': # left
        servo.write(45)


    elif direction == 'u': # front
        servo.write(0) 

    elif direction == 's': # stop
        GPIO.output(ai1, GPIO.LOW)
        GPIO.output(ai2, GPIO.LOW)

    elif direction == 'e': # exit program
        GPIO.cleanup() # must include at the end of the program to release the pins used
        break

    time.sleep(7)
    GPIO.output(ai1, GPIO.LOW)
    GPIO.output(ai2, GPIO.LOW)
    break
