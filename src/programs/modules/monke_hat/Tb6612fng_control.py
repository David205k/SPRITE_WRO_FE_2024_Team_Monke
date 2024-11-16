import RPi.GPIO as GPIO # use RPi library for controlling GPIO pins
import modules.monke_hat.Pwm_control as Pwm_control

GPIO.setwarnings(False) # turn off warnings for pins (if pins were previously used and not released properly there will be warnings)
# GPIO.setmode(GPIO.BOARD) # pin name convention used is pin numbers on board

class Motor:

    """
    Class for controlling a single motor using the TB6612FNG motor driver

    Methods
    -------
    speed(speed: float)
    """

    def __init__(self, stby: int, pwm: int, i1: int, i2: int):
        """
        Parameters
        ----------
        stby: int
            Standby pin (Physical RPI pin that is connected to it)
        pwm: int
            Pwm pin (Physical RPI pin that is connected to it)
        i1: int
            Pin 1 of the motor (Physical RPI pin that is connected to it)
        i2: int
            Pin 2 of the motor (Physical RPI pin that is connected to it)
        """

        self.stby = stby
        self.pwmA = pwm
        self.ai1 = i1
        self.ai2 = i2

        # set pins to output mode
        GPIO.setup(stby,GPIO.OUT)    
        GPIO.setup(i2,GPIO.OUT)
        GPIO.setup(i1,GPIO.OUT)

        # set up pwm signal
        self.pwmObj = Pwm_control.pwm(pwm, 100, 0)

        GPIO.output(i1, GPIO.LOW)
        GPIO.output(i2, GPIO.LOW)
        GPIO.output(stby, GPIO.HIGH)

    def speed(self, speed: float):
        """
        Set the motor speed

        Parameters
        ----------
        speed: float
            Motor speed (-100.0 <= speed <= 100.0)
        """
        max(min(speed, -100), 100) # cap speed at -100 and 100

        if speed > 0:
            GPIO.output(self.ai1, GPIO.HIGH)
            GPIO.output(self.ai2, GPIO.LOW)
        elif speed < 0:
            GPIO.output(self.ai1, GPIO.LOW)
            GPIO.output(self.ai2, GPIO.HIGH)
        else:
            GPIO.output(self.ai1, GPIO.LOW)
            GPIO.output(self.ai2, GPIO.LOW)

        self.pwmObj.setPwm(abs(speed))