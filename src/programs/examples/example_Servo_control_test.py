# need sudo pigpiod
import sys

sys.path.append('/home/monke/WRO FE 2024 (Repository)/src/programs/modules/monke_hat')

from Servo_control import Servo

servo = Servo(5, 0, -5, -60, 60)
while True:

    try:
        angle = int(input("Angle: "))
    except ValueError:
        break

    servo.write(angle)