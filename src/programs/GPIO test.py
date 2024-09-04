import RPi.GPIO as GPIO # use RPi library for controlling GPIO pins
import time
import math
from gpiozero.pins.pigpio import PiGPIOFactory

GPIO.setwarnings(False) # turn off warnings for pins (if pins were previously used and not released properly there will be warnings)
GPIO.setmode(GPIO.BOARD) # pin name convention used is pin numbers on board
factory = PiGPIOFactory()

startBut1 = 16
startBut2 = 18
GPIO.setup(startBut1,GPIO.IN)
GPIO.setup(startBut2,GPIO.IN)

def main():
    while True:
        print(f"but1: {GPIO.input(startBut1)}, but2 {GPIO.input(startBut2)}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        GPIO.cleanup()