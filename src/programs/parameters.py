# global variables
SPEED = 20              # in %
MIN_WALL_DIST = 30      # cm
TOTAL_TURNS = 12        # total number of corner turns

# upper and lower in HSV
# bbox colour in BGR
GREEN_SIGN = {"lower":(45, 100, 30), "upper":(80, 255, 200),
              "height":9.5, "width":5, 
              "type": "green", "bbox colour":(0,255,0), 
              "min h":40, "min w":20}
RED_SIGN = {"lower":(0, 100, 50), "upper":(7, 255, 255), 
             "lower2":(175, 100, 50), "upper2":(179, 255, 255),
             "height":9.5, "width":5, "type": "red", "bbox colour":(0,0,255),  
             "min h":40, "min w":20}
PARKING_LOT = {"lower":(142, 100, 50), "upper":(179, 255, 255),
                "height":10, "width":20, 
               "type": "pink", "bbox colour":(255,51,255), 
               "min h":40, "min w":80}

# old
# MAX_SPEED_CMS = 44 #cm/s
# CAR_WIDTH = 17 # cm
# CAR_LENGTH = 18.5 # cm
# WHEELBASE = 12 # cm

# new
MAX_SPEED_CMS = 44 #cm/s
CAR_WIDTH = 11.5 # cm
CAR_LENGTH = 17.5 # cm
CAR_HEIGHT = 30 # cm
WHEELBASE = 11 # cm