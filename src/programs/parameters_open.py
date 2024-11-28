from robot_config import camera

# global variables
SPEED = 25              # in %
MIN_WALL_DIST = 30      # cm
TOTAL_TURNS = 12       # total number of corner turns

BLUE_LINE = {"lower":(100, 90, 100), "upper":(130, 210, 210), "colour bgr":(200,100,0)}
ORANGE_LINE = {"lower":(5, 110, 110), "upper":(15, 255, 255), "colour bgr":(0,100,200)}

WALLS = {"lower":(0, 0, 0), "upper":(179, 255, 70), "colour bgr":(100,100,100)}

LINE_ZONE = (20, 300, camera["shape"][0]-40, camera["shape"][1]-350) # x,y,w,h
WALL_ZONE = (0, 250, camera["shape"][0], 290-250) # x,y,w,h