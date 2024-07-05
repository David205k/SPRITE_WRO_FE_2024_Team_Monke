import RPi.GPIO as GPIO # use RPi library for controlling GPIO pins

GPIO.setwarnings(False) # turn off warnings for pins (if pins were previously used and not released properly there will be warnings)
GPIO.setmode(GPIO.BOARD) # pin name convention used is pin numbers on board

class pwm:

    def __init__(self, pin, freq, startDuty):
        self.pin = pin
        self.freq = freq
        self.startDuty = startDuty,

        GPIO.setup(pin,GPIO.OUT)

        self.pwmObject = GPIO.PWM(pin, freq)
        self.pwmObject.start(startDuty)

    def setPwm(self, dutyCycle):
        self.pwmObject.ChangeDutyCycle(dutyCycle)