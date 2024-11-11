"""
This file stores the pin connections and component configurations for the robot.
May consider using a yaml file instead.
"""

camera={"shape": (640,480), "vertical FOV": 41, "horizontal FOV": 66, "focal length": 697}

# gpio pin convention
servo={"pin":5,"start":0, "offset":-5, "min":-60, "max":60}
us1={"trig":17, "echo":4}
us2={"trig":22, "echo":27}
us3={"trig":9, "echo":10}
us4={"trig":13, "echo":6}
us5={"trig":8, "echo":7}

# physcial/board pin convention
rgb={"red":8, "green":12, "blue":10}
pb={1: 16, 2: 18}
mDrvr={"stby": 37, "pwmA": 35, "ai1": 36, "ai2": 40} 

