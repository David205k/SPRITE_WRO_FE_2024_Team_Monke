import RPi.GPIO as GPIO # use RPi library for controlling GPIO pins

GPIO.setwarnings(False) 
# GPIO.setmode(GPIO.BOARD) # pin name convention used is pin numbers on board

class pwm:
    """
    Class for a Raspberry PI GPIO pwm pin

    Uses the RPi.GPIO library for controlling the pins

    Methods
    -------
    setPwm(dutyCycle)
    """

    def __init__(self, pin: int, freq: float, startDuty: float = 0):
        """
        Parameters
        ----------
        pin : int 
            Physical RPI pin to control 
        freq : float
            Frequency in hertz
        startDuty : float
            Starting duty cycle (0.0 <= dc <= 100.0) 
        """

        self.pin = pin
        self.freq = freq
        self.startDuty = startDuty

        GPIO.setup(pin,GPIO.OUT)

        self.pwmObject = GPIO.PWM(pin, freq)
        self.pwmObject.start(startDuty)

    def setPwm(self, dutyCycle: float):
        """
        Changes the duty cycle of the GPIO pin

        Parameters
        ----------
        dutycycle: float
            The new duty cycle to be assigned. (0.0 <= dc <= 100.0)
        """

        self.pwmObject.ChangeDutyCycle(dutyCycle)