from gpiozero import DistanceSensor
import time

ultrasonic = DistanceSensor(echo=6, trigger=13, max_distance=3)

try:
    while True: 
        print(ultrasonic.distance*100)

except KeyboardInterrupt:
    pass

