import sys
sys.path.append('/home/monke/WRO FE 2024 (Repository)/src/programs')

from modules.monke_hat.Tb6612fng_control import Motor

motor = Motor(26, 19, 12, 20) 

while True:

    try:
        speed = int(input("Speed: "))
    except ValueError:
        break

    motor.speed(speed)
