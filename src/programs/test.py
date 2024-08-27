import modules.ServoControl_gpiozero as myservo
from gpiozero.pins.pigpio import PiGPIOFactory

factory = PiGPIOFactory()

servo = myservo.myServo(gpioPin=5, startPos=0, minAng=-70, maxAng=70)

while True:

    angle = int(input())

    try:
        servo.write(angle)
    except KeyboardInterrupt:
        break