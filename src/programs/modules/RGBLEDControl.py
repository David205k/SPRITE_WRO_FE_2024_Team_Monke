import RPi.GPIO as GPIO
import modules.PwmControl as PwmControl

GPIO.setmode(GPIO.BOARD) #  pins were previously used and not released properly there will be warnings)
GPIO.setwarnings(False) # turn off warnings for pins (if pins were previously used and not released properly there will be warnings)

class LED:

    def __init__(self, red=8, green=12, blue=10):

        self.red = red
        self.green = green
        self.blue = blue    

        # set up rgb pins pwm
        self.pwmRed = PwmControl.pwm(self.red, 100, 0)
        self.pwmGreen = PwmControl.pwm(self.green, 100, 0)
        self.pwmBlue = PwmControl.pwm(self.blue, 100, 0)


    def rgb(self, red=0, green=0, blue=0): # set led colour
        self.pwmRed.setPwm(abs(red))
        self.pwmGreen.setPwm(abs(green))
        self.pwmBlue.setPwm(abs(blue))
    
    def off(self): # turn off led
        self.pwmRed.setPwm(0)
        self.pwmGreen.setPwm(0)
        self.pwmBlue.setPwm(0)
