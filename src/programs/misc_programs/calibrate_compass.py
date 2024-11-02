"""
Run this file to get to calibrate the HMC5883L compass module. Corresponding offset values
for each axes will be generated. Copy and replace the values in the "compass" class 
in "HMC5883lControl.py".
"""


import smbus2
import time
from tqdm import tqdm
import csv
import RPi.GPIO as GPIO # use RPi library for controlling GPIO pins

from monke_hat import Car
from component_params import *

GPIO.setwarnings(False) # turn off warnings for pins (if pins were previously used and not released properly there will be warnings)
GPIO.setmode(GPIO.BOARD) # pin name convention used is pin numbers on board

# initialise car object
car = Car.Car(
    camera=camera,
    wheelBase=wheelBase,
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

# Gather calibration data
def gather_calibration_data(samples=100):
    x_readings = []
    y_readings = []
    z_readings = []
    
    for _ in tqdm(range(samples)):
        x, y, z = read_magnetometer()
        x_readings.append(x)
        y_readings.append(y)
        z_readings.append(z)
        time.sleep(0.1)  # Wait 100 ms between readings
        
    return x_readings, y_readings, z_readings

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

# Main function
def main():
    initialize_hmc5883l()
    
    car.LED.off()

    # Wait until button press to start 
    print("Press button to start calibration.")
    while not (car.read_button()):
        continue

    # Drive in clockwise direction
    print("Starting calibration in clockwise direction.")
    print("Gathering calibration data...")
    car.motor.speed(40)
    car.servo.write(40)
    car.LED.rgb(0,0,255) # blue LED

    x_readings, y_readings, z_readings = gather_calibration_data(samples=250)

    # Stop
    car.motor.speed(0)
    car.servo.write(0)
    time.sleep(1)

    # Drive in anticlockwise direction
    print("Starting calibration in anti clockwise direction.")
    print("Gathering calibration data...")
    car.motor.speed(40)
    car.servo.write(-40)
    car.LED.rgb(255,0,0)
    x_readings2, y_readings2, z_readings2 = gather_calibration_data(samples=250)
    x_readings, y_readings, z_readings = x_readings+x_readings2, y_readings+y_readings2, z_readings+z_readings2

    car.motor.speed(0)
    car.servo.write(0)
    car.LED.off(0)

    print("Calibration complete.")
    print("Calculating offsets and scales...")

    x_offset, y_offset, z_offset, x_scale, y_scale, z_scale = calculate_offsets(x_readings, y_readings, z_readings)
    
    print(f"X_OFFSET, Y_OFFSET, Z_OFFSET = {x_offset}, {y_offset}, {z_offset}")
    print(f"X_SCALE, Y_SCALE, Z_SCALE = {x_scale}, {y_scale}, {z_scale}")

    # Uncomment to print to readings excel sheet
    # print("Storing calibration data in Excel...")
    # save_to_csv(x_readings, y_readings, z_readings)

if __name__ == "__main__": 
    main()