import RPi.GPIO as GPIO
import time
import ultrasonicSensor_control as us

# Initialize GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

# Define pin numbers
pwmA = 11   # Motor PWM
ai2 = 13    # Motor Input 2
ai1 = 15    # Motor Input 1
servo = 19  # Servo PWM

# Ultrasonic sensor pins
front_trigger_pin = 16
front_echo_pin = 18
left_trigger_pin = 22
left_echo_pin = 21
right_trigger_pin = 24
right_echo_pin = 23

# Set up motor driver pins
GPIO.setup(pwmA, GPIO.OUT)
GPIO.setup(ai2, GPIO.OUT)
GPIO.setup(ai1, GPIO.OUT)

# Set up servo motor
GPIO.setup(servo, GPIO.OUT)
servo_pwm = GPIO.PWM(servo, 50)  # PWM frequency for servo (50 Hz)
servo_pwm.start(5)  # Initial position of servo

# Set up ultrasonic sensors
front_sensor = us.ultrasonic(front_trigger_pin, front_echo_pin)
left_sensor = us.ultrasonic(left_trigger_pin, left_echo_pin)
right_sensor = us.ultrasonic(right_trigger_pin, right_echo_pin)

# Function to control servo angle
def set_servo_angle(angle):
    # Map angle (0 to 180 degrees) to PWM duty cycle (2.5 to 12.5)
    duty = angle / 18.0 + 2.5
    servo_pwm.ChangeDutyCycle(duty)

# Main control loop
while True:
    # Read distances from sensors
    front_distance = front_sensor.getDist()
    left_distance = left_sensor.getDist()
    right_distance = right_sensor.getDist()

    print(f"Front Distance: {front_distance} cm, Left Distance: {left_distance} cm, Right Distance: {right_distance} cm")

    userInput = input()

    if userInput.startswith(('f', 'b', 'r', 'l', 'u', 's')):
        direction = userInput[0]
        speed = userInput[1:]

        try:
            pi_pwm.ChangeDutyCycle(int(speed))
        except ValueError:
            speed = 0

        if direction == 'f':  # forward
            GPIO.output(ai1, GPIO.HIGH)
            GPIO.output(ai2, GPIO.LOW)

        elif direction == 'b':  # backward
            GPIO.output(ai1, GPIO.LOW)
            GPIO.output(ai2, GPIO.HIGH)

        elif direction == 'r':  # right
            set_servo_angle(90)  # Adjust angle as needed for right turn

        elif direction == 'l':  # left
            set_servo_angle(0)  # Adjust angle as needed for left turn

        elif direction == 'u':  # up or center position
            set_servo_angle(45)  # Adjust angle as needed for center position

        elif direction == 's':  # stop
            GPIO.output(ai1, GPIO.LOW)
            GPIO.output(ai2, GPIO.LOW)

    # Steering using sensor readings
    # If front sensor detects an obstacle and both left and right distances are valid
    if front_distance < 20 and left_distance > 0 and right_distance > 0:
        if left_distance > right_distance:
            set_servo_angle(0)  # Steer left
        else:
            set_servo_angle(90)  # Steer right

    elif userInput == 'e':  # exit program
        GPIO.cleanup()  # Release GPIO pins
        break

    time.sleep(0.1)  # Add a small delay to control loop for stability
