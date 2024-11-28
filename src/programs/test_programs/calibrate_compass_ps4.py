"""
Run this file to get to calibrate the HMC5883L compass module. Corresponding offset values
for each axes will be generated. Copy and replace the values in the "compass" class 
in "HMC5883lControl.py".
"""

import sys
sys.path.append('/home/monke/WRO FE 2024 (Repository)/src/programs')

import pygame
import smbus2
import time
from tqdm import tqdm
import csv
import RPi.GPIO as GPIO # use RPi library for controlling GPIO pins

from modules.monke_hat.Car import Car
from robot_config import *

GPIO.setwarnings(False) # turn off warnings for pins (if pins were previously used and not released properly there will be warnings)
# GPIO.setmode(GPIO.BOARD) # pin name convention used is pin numbers on board

# Initialize pygame and the joystick
pygame.init()
pygame.joystick.init()

# Assuming there's one controller connected
joystick = pygame.joystick.Joystick(0)
joystick.init()
print("Controller name:", joystick.get_name())

# initialise car object
car = Car(
)


# Initialize I2C bus
bus = smbus2.SMBus(1)  # 1 indicates /dev/i2c-1

# HMC5883L address and registers
HMC5883L_ADDRESS = 0x1E
CONFIG_A_REG = 0x00
MODE_REG = 0x02
DATA_OUT_X_MSB_REG = 0x03

# Initialize HMC5883L
def initialize_hmc5883l():
    # Set HMC5883L to continuous measurement mode
    bus.write_byte_data(HMC5883L_ADDRESS, CONFIG_A_REG, 0x70)  # 8-average, 15 Hz default, normal measurement
    bus.write_byte_data(HMC5883L_ADDRESS, MODE_REG, 0x00)  # Continuous measurement mode

# Read raw magnetometer data
def read_magnetometer():
    data = bus.read_i2c_block_data(HMC5883L_ADDRESS, DATA_OUT_X_MSB_REG, 6)
    x = data[0] << 8 | data[1]
    z = data[2] << 8 | data[3]
    y = data[4] << 8 | data[5]
    
    # Convert to signed 16-bit integers
    if x > 32767:
        x -= 65536
    if y > 32767:
        y -= 65536
    if z > 32767:
        z -= 65536
        
    return x, y, z

# Calculate offsets and scales
def calculate_offsets(x_readings, y_readings, z_readings):
    x_offset = (max(x_readings) + min(x_readings)) / 2
    y_offset = (max(y_readings) + min(y_readings)) / 2
    z_offset = (max(z_readings) + min(z_readings)) / 2
    
    x_scale = (max(x_readings) - min(x_readings)) / 2
    y_scale = (max(y_readings) - min(y_readings)) / 2
    z_scale = (max(z_readings) - min(z_readings)) / 2
    
    return x_offset, y_offset, z_offset, x_scale, y_scale, z_scale

# Function to save calibration data to CSV
def save_to_csv(x_readings, y_readings, z_readings, filename="calibration_data.csv"):
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        
        # Write headers
        writer.writerow(["X", "Y", "Z"])
        
        # Write data
        for x, y, z in zip(x_readings, y_readings, z_readings):
            writer.writerow([x, y, z])
    
    print(f"Calibration data saved to {filename}")

def map(var, min1, max1, min2, max2):
    return ((var - min1) / (max1-min1)) * (max2 - min2) + min2

record = False

prev_button_x = False
prev_button_o = False
prev_button_tri = False

prev_time = 0

button_x = False
button_o = False
button_tri = False

up_down_prev = 0

x_readings = []
y_readings = []
z_readings = []

if __name__ == "__main__":
    initialize_hmc5883l()

    try:
        while True:

            pygame.event.pump()

            # Access joystick and button values
            left_vertical = joystick.get_axis(1)  # Left stick vertical
            right_horizontal = joystick.get_axis(3)  # Right stick vertical

            prev_button_x = button_x
            button_x = joystick.get_button(0)  # X button
            button_o = joystick.get_button(1)  # o button
            button_tri = joystick.get_button(2) # triangle button

            dpad = joystick.get_hat(0) # dpad (list, shape is [4])

            speed = -round(map(left_vertical, -1, 1 , -50, 50))
            servoAng = -round(map(right_horizontal, -1, 1, -45, 45))

            car.motor.speed(speed)
            car.servo.write(servoAng)

            if button_x == 1 and prev_button_x == 0:
                record = not record

            if record:

                car.LED.rgb(100, 0, 0)
                print("Started recording")
                if (time.time() - prev_time) >= 0.1:
                    x, y, z = read_magnetometer()
                    x_readings.append(x)
                    y_readings.append(y)
                    z_readings.append(z)
                    prev_time = time.time()
            else:
                print("finished recording")
                car.LED.rgb(0,100,0)

            if button_tri == 1:
                break

    finally:
        car.inactive()

        print("Generatiing offsets and scales...")

        x_offset, y_offset, z_offset, x_scale, y_scale, z_scale = calculate_offsets(x_readings, y_readings, z_readings)

        print(f"X_OFFSET, Y_OFFSET, Z_OFFSET = {x_offset}, {y_offset}, {z_offset}")
        print(f"X_SCALE, Y_SCALE, Z_SCALE = {x_scale}, {y_scale}, {z_scale}")


        # Uncomment to print to readings excel sheet
        print("Storing calibration data in Excel...")
        save_to_csv(x_readings, y_readings, z_readings)

        car.LED.off()
        joystick.quit()
        pygame.quit()
        GPIO.cleanup()


