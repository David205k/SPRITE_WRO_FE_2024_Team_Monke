from robot_config import camera

# global variables
SPEED = 20              # in %
turn_dist = 100
MIN_WALL_DIST = 30      # cm
TOTAL_TURNS = 12        # total number of corner turns

# upper and lower in HSV
# bbox colour in BGR
GREEN_SIGN = {"lower":(45, 100, 30), "upper":(80, 255, 200),
              "height":9.5, "width":5, 
              "type": "green", "bbox colour":(0,255,0), 
              "min h":40, "min w":20}
RED_SIGN = {"lower":(0, 100, 50), "upper":(5, 255, 255), 
             "lower2":(175, 100, 50), "upper2":(179, 255, 255),
             "height":9.5, "width":5, "type": "red", "bbox colour":(0,0,255),  
             "min h":40, "min w":20}
PARKING_LOT = {"lower":(142, 100, 50), "upper":(179, 255, 255),
                "height":10, "width":20, 
               "type": "pink", "bbox colour":(255,51,255), 
               "min h":40, "min w":20}

BLUE_LINE = {"lower":(100, 70, 60), "upper":(130, 210, 210), "colour bgr":(200,100,0)}
ORANGE_LINE = {"lower":(5, 60, 60), "upper":(12, 255, 255), "colour bgr":(0,100,200)}

LINE_ZONE = (20, camera["shape"][1]//2, camera["shape"][0]-40, camera["shape"][1]//2) # x,y,w,h
SIGN_ZONE = (10, 210, camera["shape"][0]-20, camera["shape"][1]-210) # x,y,w,h
