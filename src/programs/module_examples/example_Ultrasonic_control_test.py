import sys
sys.path.append("/home/monke/WRO FE 2024 (Repository)/src/programs/modules/monke_hat")

from monke_hat import Ultrasonic_control as Ultrasonic

import subprocess

process = subprocess.run("sudo pigpiod", shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

front = Ultrasonic.Sensor(13, 6)
left = Ultrasonic.Sensor(22,27)
right = Ultrasonic.Sensor(8,7)

front.start()
left.start()
right.start()

try:
    while True:
        print(f"f: {front.get_distance()} l: {left.get_distance()} r: {right.get_distance()}")
except Exception:
    pass
finally:
    front.stop()
    left.stop()
    right.stop()

