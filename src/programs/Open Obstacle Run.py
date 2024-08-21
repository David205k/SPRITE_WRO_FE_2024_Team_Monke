import RPi.GPIO as GPIO
import time
from gpiozero import DistanceSensor

# Initialize GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

# Define pin numbers
pwmA = 35   # Motor PWM
ai2 = 40    # Motor Input 2
ai1 = 36    # Motor Input 1
stby = 37
servo = 29  # Servo PWM

# Set up motor driver pins
GPIO.setup(pwmA, GPIO.OUT)
GPIO.setup(ai2, GPIO.OUT)
GPIO.setup(ai1, GPIO.OUT)

# Set up servo motor
GPIO.setup(servo, GPIO.OUT)
servo_pwm = GPIO.PWM(servo, 50)  # PWM frequency for servo (50 Hz)
servo_pwm.start(6)  # Initial position of servo

# Set up PWM for motor speed control
pi_pwm = GPIO.PWM(pwmA, 100)  # PWM frequency for motor (100 Hz)
pi_pwm.start(0)  # Initial duty cycle (0%)

# Set up ultrasonic sensors
us2 = DistanceSensor(echo=27, trigger=22) #pins care gpio pins
us3 = DistanceSensor(echo=10, trigger=9) #pins are gpio pins
us4 = DistanceSensor(echo=6, trigger=13) #pins are gpio pins

# Function to control servo angle
def set_servo_angle(angle):
    # Map angle (0 to 180 degrees) to PWM duty cycle (2.5 to 12.5)
    duty = (angle / 180.0) * (9-2) + 2
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
        front_distance = us4.distance * 100
        left_distance = us2.distance * 100
        right_distance = us3.distance * 100
        
        print(f"Front Distance: {front_distance} cm, Left Distance: {left_distance} cm, Right Distance: {right_distance} cm")

        # Automatic movement logic
        if front_distance < 20:

            if left_distance > right_distance:
                set_servo_angle(30)
            else:
                set_servo_angle(60)
            pi_pwm.ChangeDutyCycle(20)  # Adjust speed as needed
        else:
            servo_pwm.ChangeDutyCycle(6)
            GPIO.output(ai1, GPIO.LOW)
            GPIO.output(ai2, GPIO.HIGH)
            pi_pwm.ChangeDutyCycle(30)  # Adjust speed as needed

        time.sleep(0.1)  # Add a small delay to control loop for stability

except KeyboardInterrupt:
    GPIO.cleanup()  # Release GPIO pins
