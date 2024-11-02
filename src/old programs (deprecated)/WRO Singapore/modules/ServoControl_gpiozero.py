from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import AngularServo
import subprocess

command = "sudo pigpiod"
process = subprocess.run(command, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

factory = PiGPIOFactory() # for interacting with raspberry pi gpio pins remotely 

class myServo:

    minAng = 0
    maxAng = 0
    offset = 0

    def __init__(self, gpioPin, startPos, offset=0, minAng=-90, maxAng=90):
        self.servo = AngularServo(gpioPin, min_angle=-90, max_angle=90, min_pulse_width=0.0004, max_pulse_width=0.0026, pin_factory=factory)
        self.write(startPos)

        self.minAng = minAng
        self.maxAng = maxAng
        self.offset = offset

    def write(self, angle):

        angle = -max(min(angle+self.offset, self.maxAng), self.minAng) # limit angle between -90 and 90
        self.servo.angle = round(angle)
    