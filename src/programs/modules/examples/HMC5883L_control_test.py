import sys
sys.path.append("/home/monke/WRO FE 2024 (Repository)/src/programs/modules/monke_hat")

from RPi import GPIO
import HMC5883L_control as HMC5883L

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
        compass = HMC5883L.compass()
    except OSError:
        print("Unable to connect to compass")
        continue 
    print("Connection to compass successful!")
    break 

while True:

    compass.set_home(GPIO.input(pb1) and GPIO.input(pb2)) # set position as home when push button is pressed
    print(compass.get_angle())