# need sudo pigpiod
import sys

sys.path.append('/home/monke/WRO FE 2024 (Repository)/src/programs')

from modules.monke_hat.Servo_control import Servo

servo = Servo(gpioPin=5, startPos=0, offset=9, minAng=-42, maxAng=60)
while True:

    try:
        angle = int(input("Angle: "))
    except ValueError:
        break

    servo.write(angle)