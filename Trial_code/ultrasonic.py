# Import the necessary libraries for GPIO control and time management
import RPi.GPIO as GPIO
import time
# Define a class for Ultrasonic Sensor control
class UltrasonicSensor:
    def __init__(self, trig, echo):
        # Initialize GPIO pins for the ultrasonic sensor
        self.trig = trig  # Trigger pin
        self.echo = echo  # Echo pin

        # Set up the GPIO mode and pins
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.trig, GPIO.OUT)
        GPIO.setup(self.echo, GPIO.IN)

    # Get the distance measured by the sensor
    def get_distance(self):
        # Ensure the trigger pin is set to LOW
        GPIO.output(self.trig, GPIO.LOW)
        time.sleep(0.2)  # Wait for the sensor to settle

        # Trigger the sensor by setting the trigger pin to HIGH for 10 microseconds
        GPIO.output(self.trig, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.trig, GPIO.LOW)

        # Wait for the echo pin to go HIGH and record the start time
        while GPIO.input(self.echo) == GPIO.LOW:
            pulse_start = time.time()

        # Wait for the echo pin to go LOW and record the end time
        while GPIO.input(self.echo) == GPIO.HIGH:
            pulse_end = time.time()

        # Calculate the duration of the pulse
        pulse_duration = pulse_end - pulse_start

        # Calculate the distance based on the pulse duration
        distance = pulse_duration * 17150  # Speed of sound is 34300 cm/s (17150 in one direction)

        return distance
