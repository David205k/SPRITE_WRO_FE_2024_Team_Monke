import RPi.GPIO as GPIO # use RPi library for controlling GPIO pins
import modules.PwmControl as PwmControl

GPIO.setwarnings(False) # turn off warnings for pins (if pins were previously used and not released properly there will be warnings)
GPIO.setmode(GPIO.BOARD) # pin name convention used is pin numbers on board

class motor:

    def __init__(self, stby, pwmA, ai1, ai2):
        self.stby = stby
        self.pwmA = pwmA
        self.ai1 = ai1
        self.ai2 = ai2

        # set pins to output mode
        GPIO.setup(stby,GPIO.OUT)    
        GPIO.setup(ai2,GPIO.OUT)
        GPIO.setup(ai1,GPIO.OUT)

        # set up pwm signal
        self.pwmAObj = PwmControl.pwm(pwmA, 100, 30)

        GPIO.output(ai1, GPIO.LOW)
        GPIO.output(ai2, GPIO.LOW)
        GPIO.output(stby, GPIO.HIGH)

    def speed(self, speed):

        if speed > 0:
            GPIO.output(self.ai1, GPIO.HIGH)
            GPIO.output(self.ai2, GPIO.LOW)
        elif speed < 0:
            GPIO.output(self.ai1, GPIO.LOW)
            GPIO.output(self.ai2, GPIO.HIGH)
        else:
            GPIO.output(self.ai1, GPIO.LOW)
            GPIO.output(self.ai2, GPIO.LOW)

        self.pwmAObj.setPwm(abs(speed))