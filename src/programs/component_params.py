"""
This file stores the pin connections and component configurations for the robot.
May consider using a yaml file instead.
"""

# Datasheets:
# Pi Cam: https://www.raspberrypi.com/documentation/accessories/camera.html
# US-015: https://downloads.autobotic.com.my/3822/3822%20US-015%20High%20Accuracy%20Ultrasonic%20Sensor.pdf
# TB6612FNG: https://www.sparkfun.com/datasheets/Robotics/TB6612FNG.pdf 


# old robot configuration
# ------------------------------------------------------------------------------------------

# camera={"shape": (640,480), "vertical FOV": 41, "horizontal FOV": 66, "focal length": 390}

# # gpio pin convention
# servo={"pin":5,"start":0, "offset":-5, "min":-60, "max":60}  # pin 29 (physical)
# us1={"trig":17, "echo":4} # 11, 7 (physical)
# us2={"trig":22, "echo":27} # 15, 13 (physical)
# us3={"trig":9, "echo":10} # 21, 19 (physical)
# us4={"trig":13, "echo":6} # 33, 31 (physical)
# us5={"trig":8, "echo":7}  # 36, 40 (physical)
# rgb={"red":14, "green":15, "blue":18}  # 8, 10, 12 (physical) 
# pb={1: 23, 2: 24} # 16, 18 (physical)
# mDrvr={"stby": 26, "pwmA": 19, "ai1": 16, "ai2": 21} # 37, 35, 36, 40  (physical)

# new robot configuration 
# -------------------------------------------------------------------------------------------

camera={"shape": (640,480), "vertical FOV": 67, "horizontal FOV": 102, "focal length": 390}

# gpio pin convention
servo={"pin":5,"start":0, "offset":8, "min":-60, "max":60} # pin 29 (physical)
us1={"trig":17, "echo":4} # 11, 7 (physical)
us2={"trig":22, "echo":27} # 15, 13 (physical)
us3={"trig":9, "echo":10} # 21, 19 (physical)
us4={"trig":13, "echo":6} # 33, 31 (physical)
us5={"trig":16, "echo":21} # 36, 40 (physical)
rgb={"red":14, "green":15, "blue":18} # 8, 10, 12 (physical)
pb={1: 23, 2: 24} # 16, 18 (physical)
mDrvr={"stby":26, "pwmA":19, "ai1":12, "ai2":20} # 37, 35, 32, 38  (physical)
