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

# Set up PWM for motor speed control
pi_pwm = GPIO.PWM(pwmA, 100)  # PWM frequency for motor (100 Hz)
pi_pwm.start(0)  # Initial duty cycle (0%)

# Set up ultrasonic sensors
front_sensor = us.ultrasonic(front_trigger_pin, front_echo_pin)
left_sensor = us.ultrasonic(left_trigger_pin, left_echo_pin)
right_sensor = us.ultrasonic(right_trigger_pin, right_echo_pin)

# Function to control servo angle
def set_servo_angle(angle):
    # Map angle (0 to 180 degrees) to PWM duty cycle (2.5 to 12.5)
    duty = angle / 18.0 + 2.5
    servo_pwm.ChangeDutyCycle(duty)

# Function to stop the vehicle
def stop_vehicle():
    GPIO.output(ai1, GPIO.LOW)
    GPIO.output(ai2, GPIO.LOW)
    pi_pwm.ChangeDutyCycle(0)

# Main control loop
try:
    while True:
        # Read distances from sensors
        front_distance = front_sensor.getDist()
        left_distance = left_sensor.getDist()
        right_distance = right_sensor.getDist()

        print(f"Front Distance: {front_distance} cm, Left Distance: {left_distance} cm, Right Distance: {right_distance} cm")

        # Automatic movement logic
        if front_distance < 20:
            stop_vehicle()
            if left_distance > right_distance:
                set_servo_angle(0)  # Steer left
            else:
                set_servo_angle(90)  # Steer right
            pi_pwm.ChangeDutyCycle(50)  # Adjust speed as needed
        else:
            set_servo_angle(45)  # Center position
            GPIO.output(ai1, GPIO.HIGH)
            GPIO.output(ai2, GPIO.LOW)
            pi_pwm.ChangeDutyCycle(50)  # Adjust speed as needed

        time.sleep(0.1)  # Add a small delay to control loop for stability

except KeyboardInterrupt:
    GPIO.cleanup()  # Release GPIO pins
