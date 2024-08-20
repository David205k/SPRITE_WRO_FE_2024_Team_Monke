from gpiozero import DistanceSensor
import time

ultrasonic = DistanceSensor(echo=27, trigger=22)

try:
    while True: 
        print(ultrasonic.distance*100)
        time.sleep(0.3)

except KeyboardInterrupt:
    pass

