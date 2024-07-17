import RPi.GPIO as GPIO

# Disable GPIO warnings and set up GPIO mode
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

class pwm:
    def __init__(self, pin, freq, startDuty):
        self.pin = pin
        self.freq = freq
        self.startDuty = startDuty

        GPIO.setup(self.pin, GPIO.OUT)  # Set up the pin as output
        self.pwm = GPIO.PWM(self.pin, self.freq)  # Initialize PWM on the pin with the given frequency
        self.pwm.start(self.startDuty)  # Start PWM with the given duty cycle

    def setPwm(self, duty):
        self.pwm.ChangeDutyCycle(duty)  # Change the duty cycle to the given value
