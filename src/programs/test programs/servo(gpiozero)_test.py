# need sudo pigpiod

from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import AngularServo

factory = PiGPIOFactory()
servo = AngularServo(5, min_angle=-90, max_angle=90, min_pulse_width=0.0004, max_pulse_width=0.0026, pin_factory=factory)

while True:

    try:
        angle = int(input("Angle: "))
    except ValueError:
        break

    servo.angle = angle