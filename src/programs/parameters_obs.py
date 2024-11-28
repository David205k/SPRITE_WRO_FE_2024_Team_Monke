from robot_config import camera

# global variables
SPEED = 20              # in %
turn_dist = 80
MIN_WALL_DIST = 30      # cm
TOTAL_TURNS = 12        # total number of corner turns

# upper and lower in HSV
# bbox colour in BGR
GREEN_SIGN = {"lower":(45, 100, 30), "upper":(80, 255, 200),
              "height":10, "width":5, 
              "type": "green", "bbox colour":(0,255,0), 
              "min h":60, "min w":30}
RED_SIGN = {"lower":(0, 100, 50), "upper":(3, 255, 255), 
             "lower2":(175, 100, 50), "upper2":(179, 255, 255),
             "height":10, "width":5, "type": "red", "bbox colour":(0,0,255),  
             "min h":60, "min w":30}
PARKING_LOT = {"lower":(142, 100, 50), "upper":(179, 255, 255),
                "height":10, "width":20, 
               "type": "pink", "bbox colour":(255,51,255), 
               "min h":60, "min w":30}
SIGN_ZONE = (10, 270, camera["shape"][0]-20, camera["shape"][1]-270) # x,y,w,h

BLUE_LINE = {"lower":(100, 70, 60), "upper":(130, 210, 210), "colour bgr":(200,100,0)}
ORANGE_LINE = {"lower":(5, 60, 60), "upper":(15, 255, 255), "colour bgr":(0,100,200)}
LINE_ZONE = (20, 350, 
             300,
            #  camera["shape"][0]-40,
               camera["shape"][1]-350) # x,y,w,h

WALLS = {"lower":(0, 0, 0), "upper":(179, 255, 65), 
          "colour bgr":(100,100,100)}
WALL_ZONE = (0, 300, camera["shape"][0], camera["shape"][1]-300) # x,y,w,h