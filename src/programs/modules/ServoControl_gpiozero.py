from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import AngularServo
import subprocess

command = "sudo pigpiod"
process = subprocess.run(command, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

factory = PiGPIOFactory() # for interacting with raspberry pi gpio pins remotely 

class myServo:

    def __init__(self, gpioPin, startPos):
        self.servo = AngularServo(gpioPin, min_angle=-90, max_angle=90, min_pulse_width=0.0004, max_pulse_width=0.0026, pin_factory=factory)
        self.write(startPos)

    def write(self, angle):

        angle = max(min(angle-7, 90), -90) # limit angle between -90 and 90
        self.servo.angle = angle
    