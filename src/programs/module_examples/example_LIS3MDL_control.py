import sys
sys.path.append("/home/monke/WRO FE 2024 (Repository)/src/programs")
from modules.monke_hat.LIS3MDL_control import Compass
from RPi import GPIO

GPIO.setwarnings(False)

def main():

    myCompass = Compass()

    pb1 = 23
    pb2 = 24
    GPIO.setup(pb1, GPIO.IN)
    GPIO.setup(pb2, GPIO.IN)

    while True:

        myCompass.set_home(GPIO.input(pb1) and GPIO.input(pb2))
        print(round(myCompass.get_angle()))
        print
    
if __name__ == "__main__":
    main()