import sys
sys.path.append("/home/monke/WRO FE 2024 (Repository)/src/programs")
sys.path.append("/home/monke/WRO FE 2024 (Repository)/src/programs/modules/monke_hat")

import Car
from component_params import *
import cv2
from RPi import GPIO

car = Car.Car(
    camera=camera,
    servo=servo,
    us_front=us4,
    us_left=us2,
    us_right=us5,
    us_spare1=us1,
    us_spare2=us3,
    rgb=rgb,
    pb=pb,
    mDrvr=mDrvr
)
def main():
    while True:

        car.compass.set_home(car.read_button())

        car.read_sensors()
        car.print_sensor_vals()

if __name__ == "__main__":
    try:
        main()
    except Exception:
        cv2.destroyAllWindows()
        GPIO.cleanup()
