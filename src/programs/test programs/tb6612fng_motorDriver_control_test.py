# example on using tb6612fng motor driver to control a DC Motor
# by David Lim 7/6/2024 Friday
import RPi.GPIO as GPIO # use RPi library for controlling GPIO pins

from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import AngularServo

GPIO.setwarnings(False) # turn off warnings for pins (if 1if pins were previously used and not released properly there will be warnings)
GPIO.setmode(GPIO.BOARD) # pin name convention used is pin numbers on board

servoPin= 29

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

factory = PiGPIOFactory()
servo = AngularServo(5, min_angle=-90, max_angle=90, min_pulse_width=0.0004, max_pulse_width=0.0026, pin_factory=factory)

# # set up pwm signal
pi_pwm = GPIO.PWM(pwmA, 100) # set pwm frequency to 100
pi_pwm.start(30) # set duty cycle to 30

# turn off pins at the start
GPIO.output(ai1, GPIO.LOW)
GPIO.output(ai2, GPIO.LOW)

GPIO.output(stby, GPIO.HIGH)

# # set up pwm signal
# servo_pwm = GPIO.PWM(servo, 50) # set pwm frequency to 50
# servo_pwm.start(5) # set duty cycle to 5

while True:


    userInput = input()
    direction, speed = userInput[0], userInput[1:]

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
        servo.angle = -45 - 7
        # angle = 45
        # duty = (angle / 180.0) * (10-2) + 2
        # servo_pwm.ChangeDutyCycle(duty)

    elif direction == 'l': # left
        servo.angle = 45 - 7
        # angle = 135
        # duty = (angle / 180.0) * (10-2) + 2
        # servo_pwm.ChangeDutyCycle(duty)

    elif direction == 'u': # front
        servo.angle = 0 - 6
        # servo_pwm.ChangeDutyCycle(6)

    elif direction == 's': # stop
        GPIO.output(ai1, GPIO.LOW)
        GPIO.output(ai2, GPIO.LOW)

    elif direction == 'e': # exit program
        GPIO.cleanup() # must include at the end of the program to release the pins used
        break