from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import AngularServo
import subprocess

command = "sudo pigpiod"
process = subprocess.run(command, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

factory = PiGPIOFactory() # for interacting with raspberry pi gpio pins remotely 

class Servo:
    """
    Class for controlling a servo

    Uses gpiozero PiGPIOFactory for accurate pwm control

    Methods
    -------
    write(angle)
    """

    def __init__(self, gpioPin: int, startPos: int, offset: int =0, minAng:int =-90, maxAng: int =90):
        """
        Parameters
        ----------

        gpioPin: int
            GPIO pin number that the servo is connected to
        startPos: int 
            Initial position of the servo (-90 to 90)
        offset: int
            Angular error from 0 degrees position (-90 to 90)
        minAng: int
            Minimum position of the servo (degrees, -90 to 90)
        maxAng: int
            Maximum position of the servo (degrees, -90 to 90)
        """

        self.servo = AngularServo(gpioPin, min_angle=-90, max_angle=90, min_pulse_width=0.0004, max_pulse_width=0.0026, pin_factory=factory)

        self.minAng = minAng
        self.maxAng = maxAng
        self.offset = offset
        self.write(startPos)

    def write(self, angle: int):
        """
        Turn servo to desired position

        Parameters
        ----------
        angle: int
            Angular position to move the servo to (-90 to 90)
        """

        angle = max(min(angle+self.offset, self.maxAng), self.minAng) # limit angle between minAng and maxAng
        angle = -round(angle)
        # print(angle) 
        self.servo.angle = angle
    