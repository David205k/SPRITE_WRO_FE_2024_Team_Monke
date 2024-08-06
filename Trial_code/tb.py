# Import the necessary libraries for GPIO control and time management
import RPi.GPIO as GPIO
import time

# Define a class for the Motor control
class Motor:
    def __init__(self, pwmA, ai1, ai2):
        # Initialize GPIO pins for the motor
        self.pwmA = pwmA  # PWM pin
        self.ai1 = ai1    # Control pin 1
        self.ai2 = ai2    # Control pin 2

        # Set up the GPIO mode and pins
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pwmA, GPIO.OUT)
        GPIO.setup(self.ai1, GPIO.OUT)
        GPIO.setup(self.ai2, GPIO.OUT)

        # Initialize PWM on the PWM pin with a frequency of 1000Hz
        self.pwm = GPIO.PWM(self.pwmA, 1000)
        self.pwm.start(0)  # Start PWM with 0% duty cycle (motor off)

    # Set the speed of the motor
    def set_speed(self, speed):
        if speed > 0:
            GPIO.output(self.ai1, GPIO.HIGH)
            GPIO.output(self.ai2, GPIO.LOW)
        elif speed < 0:
            GPIO.output(self.ai1, GPIO.LOW)
            GPIO.output(self.ai2, GPIO.HIGH)
        else:
            GPIO.output(self.ai1, GPIO.LOW)
            GPIO.output(self.ai2, GPIO.LOW)

        # Set the PWM duty cycle based on the absolute value of speed
        self.pwm.ChangeDutyCycle(abs(speed))

    # Stop the motor
    def stop(self):
        # Set the control pins to LOW and stop PWM
        GPIO.output(self.ai1, GPIO.LOW)
        GPIO.output(self.ai2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(0)
