import sys
sys.path.append("/home/monke/WRO FE 2024 (Repository)/src/programs")

from modules.monke_hat.Car import Car
from robot_config import *
import cv2
from RPi import GPIO

car = Car()

def main():
    while True:

        car.compass.set_home(car.read_button())

        car.read_sensors(True)
        car.turn(-90)
        car.motor.speed(10)

if __name__ == "__main__":
    # try:
        main()
    # except Exception as E:
    #     print("Exception ", E)
    # finally:
        cv2.destroyAllWindows()
        GPIO.cleanup()
