import sys
sys.path.append('/home/monke/WRO FE 2024 (Repository)/src/programs/modules/monke_hat')

from Tb6612fng_control import Motor

motor = Motor(37, 35, 36, 40) 

while True:

    try:
        speed = int(input("Speed: "))
    except ValueError:
        break

    motor.speed(speed)
