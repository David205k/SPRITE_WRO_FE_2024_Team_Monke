import sys
sys.path.append("/home/monke/WRO FE 2024 (Repository)/src/programs")

from modules.monke_hat.Car import Car
from robot_config import *
from modules.PS4_control import RC_control

from tqdm import tqdm

import time
from modules.monke_hat.LIS3MDL_control import Compass

MAX_SPEED = 40 # maximum motor speed
MAX_ANGLE = 45 # maximum motor angle

car = Car()

def map(var, min1, max1, min2, max2):
    return ((var - min1) / (max1-min1)) * (max2 - min2) + min2

def main():

    controller = RC_control()
    
    sensor = Compass()

    x_readings, y_readings, z_readings = [], [], []

    car.inactive()
    while True:

        controller.read_controller()
        controller.write_to_car(car, max_speed=50, max_angle=45)

        # read raw compass values
        if controller.buttons["X"][2]: # if X state is on
            # record readings
            car.LED.rgb(0,0,255)
            while True:
                try:
                    x, y, z = sensor.lis3mdl._raw_mag_data
                    x_readings.append(x)
                    y_readings.append(y)
                    z_readings.append(z)
                    print(f"X: {x}, Y: {y}, Z: {z}")
                except OSError:
                    print("cannot connect to compass")
                    continue
                break
        else:
            car.LED.off()

        if controller.buttons["TRI"][1]: # if triangle is pressed
            break
                
    car.inactive()

    sensor.calculate_offsets(x_readings ,y_readings ,z_readings )
    
    # Uncomment to print to readings excel sheet
    sensor.save_to_csv(x_readings, y_readings, z_readings)


if __name__ == "__main__":

    main()
