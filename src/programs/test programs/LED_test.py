import RGB_LED_control as RGB
import RPi.GPIO as GPIO # use RPi library for controlling GPIO pins

GPIO.setwarnings(False) # turn off warnings for pins (if 1if pins were previously used and not released properly there will be warnings)
GPIO.setmode(GPIO.BOARD) #  pins wereframe.shape[1] > green.X > frame.shape[1]//2 previously used and not released properly there will be warnings)

LED = RGB.LED(8, 12, 10)

while True:

    userInput = input()

    try:
        red, blue, green = userInput.split()
        LED.rgb(int(red), int(green), int(blue))
    except ValueError:
        red, green, blue = 0, 0, 0

    if userInput == 'q':
        GPIO.cleanup() # must include at the end of the program to release the pins used
        break



