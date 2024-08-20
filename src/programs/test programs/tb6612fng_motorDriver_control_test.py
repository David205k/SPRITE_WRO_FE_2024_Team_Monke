# example on using tb6612fng motor driver to control a DC Motor
# by David Lim 7/6/2024 Friday
import RPi.GPIO as GPIO # use RPi library for controlling GPIO pins
import ultrasonicSensor_control as us

GPIO.setwarnings(False) # turn off warnings for pins (if 1if pins were previously used and not released properly there will be warnings)
GPIO.setmode(GPIO.BOARD) #  pins were previously used and not released properly there will be warnings)

servo = 29

# motor driver connections
pwmA = 35
ai2 = 40
ai1 = 36
stby = 37

GPIO.setmode(GPIO.BOARD) # pin name convention used is pin numbers on board

# set pins to output mode
GPIO.setup(pwmA,GPIO.OUT)
GPIO.setup(ai2,GPIO.OUT)
GPIO.setup(ai1,GPIO.OUT)
GPIO.setup(servo,GPIO.OUT)
GPIO.setup(stby,GPIO.OUT)

# set up pwm signal
pi_pwm = GPIO.PWM(pwmA, 100) # set pwm frequency to 100
pi_pwm.start(30) # set duty cycle to 30

# turn off pins at the start
GPIO.output(ai1, GPIO.LOW)
GPIO.output(ai2, GPIO.LOW)

GPIO.output(stby, GPIO.HIGH)

# set up pwm signalus1 = us.ultrasonic(16, 18)
servo_pwm = GPIO.PWM(servo, 50) # set pwm frequency to 50
servo_pwm.start(5) # set duty cycle to 5

us1 = us.ultrasonic(16, 18)

while True:

    #print(us1.getDist())

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
        servo_pwm.ChangeDutyCycle(6)

    elif direction == 'l': # left
        servo_pwm.ChangeDutyCycle(2)

    elif direction == 'u': # left
        servo_pwm.ChangeDutyCycle(4.5)

    elif direction == 's': # stop
        GPIO.output(ai1, GPIO.LOW)
        GPIO.output(ai2, GPIO.LOW)

    elif direction == 'e': # exit program
        GPIO.cleanup() # must include at the end of the program to release the pins used
        break