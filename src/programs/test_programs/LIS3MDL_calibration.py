import sys
sys.path.append("/home/monke/WRO FE 2024 (Repository)/src/programs")
from modules.monke_hat.Car import Car
from robot_config import *

from tqdm import tqdm
import csv
import pygame

import time
import board, adafruit_lis3mdl

MAX_SPEED = 40 # maximum motor speed
MAX_ANGLE = 45 # maximum motor angle

car = Car()


# Gather calibration data
def gather_calibration_data(_sensor, samples=100):
    x_readings = []
    y_readings = []
    z_readings = []
    
    for _ in tqdm(range(samples)):
        x, y, z = _sensor.magnetic
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
        
        writer.writerow(["X", "Y", "Z"]) # Write headers
        
        for x, y, z in zip(x_readings, y_readings, z_readings): # Write data
            writer.writerow([x, y, z])
    
    print(f"Calibration data saved to {filename}")

def map(var, min1, max1, min2, max2):
    return ((var - min1) / (max1-min1)) * (max2 - min2) + min2

def main(manual=False):

    i2c = board.I2C()  # uses board.SCL and board.SDA
    sensor = adafruit_lis3mdl.LIS3MDL(i2c)

    if not manual:

        # car.inactive()

        # # Wait for button press to start
        # print("Press button to start calibration.\nCar will drive CCW then CW.")
        # while not car.read_button():
        #     continue

        # print ("Gathering data in CCW direction.")    
        # car.LED.rgb(100,0,0)
        # car.motor.speed(20)
        # car.turn(40)
        # x_readings, y_readings, z_readings = gather_calibration_data(sensor, 200)

        # car.inactive()
        # time.sleep(1)
        # car.motor.speed(20)
        # car.turn(12)   
        # time.sleep(3) 

        # print ("Gathering data in CW direction.")    
        # car.LED.rgb(0,0,100)
        # car.motor.speed(20)
        # car.turn(-40)    
        # x2, y2, z2 = gather_calibration_data(sensor, 200)

        # car.inactive() 
        # print ("Finished collecting data.")   
        # print ("Calculating offsets and scales...")   

        # x_readings, y_readings, z_readings += x2, y2, z2
        pass
    else:
        # Initialize pygame and the joystick
        pygame.init()
        pygame.joystick.init()

        # Assuming there's one controller connected
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print("Controller name:", joystick.get_name())
        
        x_readings,y_readings,z_readings = [], [], []
        print("Press x to collect data and again to stop.")
        print("Press triangle to exit.")
        prev_button_x = False
        prev_button_o = False
        prev_button_tri = False

        button_x = False
        button_o = False
        button_tri = False

        gather_data = False
        car.inactive()
        while True:
            pygame.event.pump()

            # Access joystick and button values
            left_vertical = joystick.get_axis(1)  # Left stick vertical
            right_horizontal = joystick.get_axis(3)  # Right stick vertical

            prev_button_x = button_x
            prev_button_o = button_o
            button_x = joystick.get_button(0)  # X button
            button_o = joystick.get_button(1)  # o button
            button_tri = joystick.get_button(2) # triangle button

            dpad = joystick.get_hat(0) # dpad (list, shape is [4])

            speed = -round(map(left_vertical, -1, 1 , -MAX_SPEED, MAX_SPEED))
            servoAng = -round(map(right_horizontal, -1, 1, -MAX_ANGLE, MAX_ANGLE))

            car.motor.speed(speed)
            car.servo.write(servoAng)

            if button_x == 1 and prev_button_x == 0:
                gather_data = not gather_data
                if gather_data:
                    print("Gathering...")
                else:
  
                    print("Stopped gathering")

            if gather_data:
                car.LED.rgb(0,0,255)
                x, y, z = sensor.magnetic
                x_readings.append(x)
                y_readings.append(y)
                z_readings.append(z)
            else:
                car.LED.rgb(0,0,0)

            if button_tri == 1:
                break

    if x_readings and y_readings and z_readings:   
        x_offset, y_offset, z_offset, x_scale, y_scale, z_scale = calculate_offsets(x_readings, y_readings, z_readings)

        print(f"X_OFFSET, Y_OFFSET, Z_OFFSET = {x_offset}, {y_offset}, {z_offset}")
        print(f"X_SCALE, Y_SCALE, Z_SCALE = {x_scale}, {y_scale}, {z_scale}")
    
    # Uncomment to print to readings excel sheet
    print("Storing calibration data in Excel...")
    save_to_csv(x_readings, y_readings, z_readings)
            

if __name__ == "__main__":

    main(manual=True)
