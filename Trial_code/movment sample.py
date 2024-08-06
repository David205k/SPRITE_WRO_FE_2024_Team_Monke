# Import necessary libraries for GPIO control, time management, motor control, ultrasonic sensors, and magnetometer
import RPi.GPIO as GPIO
import time
from tb6612fng import Motor
from ultrasonicSensor_control import UltrasonicSensor
from magnetometer_control import Magnetometer

# Initialize the motors and sensors
motor_left = Motor(pwmA=12, ai1=18, ai2=16)
motor_right = Motor(pwmA=33, ai1=36, ai2=32)
front_sensor = UltrasonicSensor(trig=7, echo=11)
left_sensor = UltrasonicSensor(trig=13, echo=15)
right_sensor = UltrasonicSensor(trig=29, echo=31)
magnetometer = Magnetometer()

# Function to move the robot forward
def move_forward():
    motor_left.set_speed(50)
    motor_right.set_speed(50)

# Function to stop the robot
def stop():
    motor_left.stop()
    motor_right.stop()

# Function to turn the robot left
def turn_left():
    motor_left.set_speed(-30)
    motor_right.set_speed(30)
    time.sleep(0.5)  # Adjust the sleep time based on the required turn angle
    stop()

# Function to turn the robot right
def turn_right():
    motor_left.set_speed(30)
    motor_right.set_speed(-30)
    time.sleep(0.5)  # Adjust the sleep time based on the required turn angle
    stop()

# Function to handle a green pillar (left turn)
def handle_green():
    move_forward()
    time.sleep(1)  # Adjust based on the distance to the pillar
    turn_left()
    move_forward()
    time.sleep(1)  # Adjust based on the distance to clear the pillar
    stop()

# Function to handle a red pillar (right turn)
def handle_red():
    move_forward()
    time.sleep(1)  # Adjust based on the distance to the pillar
    turn_right()
    move_forward()
    time.sleep(1)  # Adjust based on the distance to clear the pillar
    stop()

# Function to handle a green pillar during lap 3 (left turn)
def handle_green_lap3():
    move_forward()
    time.sleep(1)  # Adjust based on the distance to the pillar
    turn_left()
    move_forward()
    time.sleep(1)  # Adjust based on the distance to clear the pillar
    stop()

# Function to handle a red pillar during lap 3 (right turn)
def handle_red_lap3():
    move_forward()
    time.sleep(1)  # Adjust based on the distance to the pillar
    turn_right()
    move_forward()
    time.sleep(1)  # Adjust based on the distance to clear the pillar
    stop()

# Function to check for obstacles and take appropriate action
def check_obstacles():
    front_distance = front_sensor.get_distance()
    left_distance = left_sensor.get_distance()
    right_distance = right_sensor.get_distance()

    # If an obstacle is detected in front, turn to the side with more space
    if front_distance < 10:
        if left_distance > right_distance:
            turn_left()
        else:
            turn_right()

# Function to execute a lap with given parameters
def execute_lap(lap_number, last_pillar_color, pillars):
    for _ in range(pillars):  # Loop through the number of pillars
        check_obstacles()
        # Placeholder for pillar detection logic (e.g., using camera)
        pillar_color = detect_pillar_color()
        if pillar_color == "green":
            handle_green() if lap_number < 3 else handle_green_lap3()
        elif pillar_color == "red":
            handle_red() if lap_number < 3 else handle_red_lap3()
        move_forward()
        time.sleep(1)  # Adjust based on the distance between pillars

    # If it's the second lap and the last pillar is red, reverse direction for the next lap
    if lap_number == 2 and last_pillar_color == "red":
        reverse_direction()

# Function to reverse the direction of movement
def reverse_direction():
    global motor_left, motor_right
    motor_left, motor_right = motor_right, motor_left  # Swap motors for reverse direction

# Function to detect the color of a pillar (placeholder)
def detect_pillar_color():
    # Placeholder for actual pillar color detection logic
    return "green"  # or "red"
