import RPi.GPIO as GPIO

# Set the GPIO mode (you can choose either GPIO.BOARD or GPIO.BCM)
GPIO.setmode(GPIO.BOARD)  # or GPIO.setmode(GPIO.BOARD)

# Release all GPIO pins
GPIO.cleanup()

print("All GPIO pins have been released.")