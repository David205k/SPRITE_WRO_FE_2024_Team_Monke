# need sudo pigpiod
import sys

sys.path.append('/home/monke/WRO FE 2024 (Repository)/src/programs')

from modules.monke_hat.RGB_LED_control import LED

LED = LED(8, 12, 10)  
while True:

    try:
        r, g, b = input("Colour: ").split()
    except ValueError:
        break

    LED.rgb(int(r),int(g),int(b))