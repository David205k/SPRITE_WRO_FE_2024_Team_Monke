"""
Changing i2c frequency for RPI:
- Add this to/boot/firmware/config.txt, "dtparam=i2c_arm=on,i2c_arm_baudrate=400000"
- Reboot pi
"""

import sys
sys.path.append("/home/monke/WRO FE 2024 (Repository)/src/programs")

from RPi import GPIO
from modules.monke_hat.HMC5883L_control import Compass

GPIO.setwarnings(False) # turn off warnings for pins (if pins were previously used and not released properly there will be warnings)
GPIO.setmode(GPIO.BOARD) # pin name convention used is pin numbers on board

# add your push button pin here
# 2 pins used here cause pin voltage a bit unstable
pb1 = 16
pb2 = 18
GPIO.setup(pb1,GPIO.IN)
GPIO.setup(pb2,GPIO.IN)

while True:  # try until it connects
    try:
        my_compass = Compass()
    except OSError:
        print("Unable to connect to compass")
        continue 
    print("Connection to compass successful!")
    break 

while True:

    my_compass.set_home(GPIO.input(pb1) and GPIO.input(pb2)) # set position as home when push button is pressed
    print(my_compass.get_angle())