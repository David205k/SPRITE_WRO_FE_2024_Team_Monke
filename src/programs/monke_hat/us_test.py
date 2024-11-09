from UltrasonicControlpigpio import Sensor

import subprocess

command = "sudo pigpiod"
process = subprocess.run(command, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

front = Sensor(13, 6)
left = Sensor(22,27)
right = Sensor(8,7)

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

