# global variables
SPEED = 20              # in %
MIN_WALL_DIST = 30      # cm
TOTAL_TURNS = 12        # total number of corner turns

GREEN_SIGN = {"lower":(45, 100, 30), "upper":(80, 255, 200), "height":9.5, "width":5, 
              "type": "green", "bbox colour":(0,255,0),
              "min h":40, "min w":20}
RED_SIGN = {"lower":(0, 100, 50), "upper":(7, 255, 255), "lower2":(175, 100, 50), "upper2":(179, 255, 255),
             "height":9.5, "width":5, "type": "red", "bbox colour":(0,0,255),
             "min h":40, "min w":20}
PARKING_LOT = {"lower":(142, 100, 50), "upper":(179, 255, 255), "height":10, "width":20, 
               "type": "pink", "bbox colour":(255,51,255),
               "min h":40, "min w":80}

MAX_SPEED_CMS = 44 #cm/s

CAR_WIDTH = 17 # cm
CAR_LENGTH = 18.5 # cm
WHEELBASE = 12 # centimeters