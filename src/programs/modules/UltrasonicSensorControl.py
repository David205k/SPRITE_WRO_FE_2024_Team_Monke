import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

class ultrasonic:

    def __init__(self, trig, echo):
        self.trig = trig
        self.echo = echo

        GPIO.setup(trig, GPIO.OUT)
        GPIO.setup(echo, GPIO.IN)

    def getDist(self):

        GPIO.output(self.trig, GPIO.LOW)
        time.sleep(0.0002)
        GPIO.output(self.trig, GPIO.HIGH)
        time.sleep(0.001)
        GPIO.output(self.trig, GPIO.LOW)


        while GPIO.input(self.echo) == 0:
            start_time = time.time()

        while GPIO.input(self.echo) == 1:
            end_time = time.time()

        duration = end_time - start_time

        distance = round(duration * 17150, 2)

        return distance