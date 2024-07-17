import time
import pwmControl

# Initialize servo and motor
servo = pwmControl.pwm(19, 50, 4.5)

# Function for left turn
def turn_left():
    servo.setPwm(4)  # Turn left
    time.sleep(0.5)
    servo.setPwm(5)  # Return to center

# Function for right turn
def turn_right():
    servo.setPwm(6)  # Turn right
    time.sleep(0.5)
    servo.setPwm(5)  # Return to center

# Function for handling green pillar
def handle_green():
    turn_left()  # Turn left
    time.sleep(0.5)
    turn_right()  # Correct to the right

# Function for handling red pillar
def handle_red():
    turn_right()  # Turn right
    time.sleep(0.5)
    turn_left()  # Correct to the left

# Function for handling green pillar in the third lap when continuing anti-clockwise
def handle_green_third_lap():
    turn_left()  # Turn left (same as usual)

# Function for handling red pillar in the third lap when reversing direction to clockwise
def handle_red_third_lap():
    turn_right()  # Turn right (same as usual)
