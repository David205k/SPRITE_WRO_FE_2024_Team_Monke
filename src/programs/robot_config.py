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

# MAX_SPEED_CMS = 44 #cm/s
# CAR_WIDTH = 17 # cm
# CAR_LENGTH = 18.5 # cm
# WHEELBASE = 12 # cm

# new robot configuration 
# -------------------------------------------------------------------------------------------

camera={"shape": (640,480), "vertical FOV": 67, "horizontal FOV": 102, "focal length": 390}

# gpio pin convention
servo={"pin":5,"start":0, "offset":9, "min":-42, "max":60} # pin 29 (physical)
us_front={"trig":17, "echo":4} # 11, 7 (physical)
us_left={"trig":22, "echo":27} # 15, 13 (physical)
us_right={"trig":9, "echo":10} # 21, 19 (physical)
us_spare1={"trig":13, "echo":6} # 33, 31 (physical)
us_spare2={"trig":16, "echo":21} # 36, 40 (physical)
rgb={"red":14, "green":18, "blue":15} # 8, 12, 10 (physical)
pb={1: 23, 2: 24} # 16, 18 (physical)
mDrvr={"stby":26, "pwmA":19, "ai1":12, "ai2":20} # 37, 35, 32, 38  (physical)
tof_left = {"x shut":11} # 23 (physical)
tof_right = {"x shut":8} # 24 (physical)
tof_back = {"x shut":25} # 22 (physical)

MAX_SPEED_CMS = 24.5 #cm/s
CAR_WIDTH = 11.5 # cm
CAR_LENGTH = 17.5 # cm
CAR_HEIGHT = 30 # cm
WHEELBASE = 11 # cm

FRONT_2_TOF_LEN =10 # cm

# coordinates of servo axis w.r.t top left pivot as origin 
SERVO_CTR = (2.7, 1.05) #(3,1.5) # cm
# yoke length
YOKE = 5 # cm
# steering link length
ANGLED_LINK = 2.5 # cm
DIST_BTW_PIVOTS = 6 # cm
ANGLED_LINK_ANG_OFFSET = 78.5 # degrees

