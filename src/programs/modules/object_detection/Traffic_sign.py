import sys
sys.path.append("/home/monke/WRO FE 2024 (Repository)/src/programs")

import cv2
from math import * 
import numpy as np
from robot_config import camera

CAM_HEIGHT = camera["shape"][1]
CAM_WIDTH = camera["shape"][0]
MIDDLE = CAM_WIDTH / 2
HOR_FOV = camera["horizontal FOV"]
VER_FOV = camera["vertical FOV"]

class Traffic_sign:

    def __init__(self, sign_params, sign_zone=None):
        self.lower_bound = sign_params["lower"]
        self.upper_bound = sign_params["upper"]
        self.width = sign_params["width"]
        self.height = sign_params["height"]
        self.type = sign_params["type"]
        self.bbox_colour = sign_params["bbox colour"]
        self.min_pixel_h = sign_params["min h"]
        self.min_pixel_w = sign_params["min w"]
        # self.bbox_ratio = self.width/self.height

        self.mask = None
        self.have_second_range = True

        try: 
            self.lower_bound2 = sign_params["lower2"]
            self.upper_bound2 = sign_params["upper2"]
        except KeyError:
            self.have_second_range = False

        if sign_zone is not None:
            self.zone = sign_zone

        self.x1 = -999
        self.y1 = -999
        self.x2 = -999
        self.y2 = -999
        self.x = -999
        self.y = -999
        self.w = -999
        self.h = -999
        self.map_x = -999
        self.map_y = -999
        self.have_sign = False

    def get_bbox(self, mask, min_pixel_h, min_pixel_w):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        largest_area = 0
        largest_bbox = None

        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            area = w * h

            if area > largest_area and h > min_pixel_h and w > min_pixel_w:

                largest_area = area
                largest_bbox = (x, y, w, h)

        return largest_bbox

    def detect_sign(self, frame, car_ang_from_heading=0):
        
        hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)   # Convert image from BGR to HSV
        mask = cv2.inRange(hsvImage, self.lower_bound, self.upper_bound)  # Create mask for the color
    
        # apply second range
        if self.have_second_range:
            mask = cv2.bitwise_or(mask,  cv2.inRange(hsvImage, self.lower_bound2, self.upper_bound2))

        self.mask = mask

        bbox = self.get_bbox(mask, self.min_pixel_h, self.min_pixel_w)

        if bbox is not None: 
            x1, y1, w, h = bbox
            x2, y2  = x1 + w, y1 + h
            x, y = x1 + w/2, y1 + h/2
    
            map_y = (self.height/h) * camera["focal length"] # get distance from object
            map_x = ((x - MIDDLE) / (CAM_WIDTH/2)) * (tan(radians(HOR_FOV/2))*map_y) # get lateral distance relative to center
            # print(f"Actual: {(map_x, map_y)}")
            # map_x, map_y = self.apply_orientation_to_obj_position(car_ang_from_heading, map_x, map_y)

            self.x1 = x1
            self.y1 = y1
            self.x2 = x2
            self.y2 = y2
            self.x = x
            self.y = y
            self.w = w
            self.h = h
            self.map_x = map_x
            self.map_y = map_y
            self.have_sign = True
        else:

            self.x1 = -999
            self.y1 = -999
            self.x2 = -999
            self.y2 = -999
            self.x = -999
            self.y = -999
            self.w = -999
            self.h = -999
            self.map_x = -999
            self.map_y = -999
            self.have_sign = False

    def apply_orientation_to_obj_position(self, car_ang_from_heading_rad, map_x, map_y):
        
        theta = radians(car_ang_from_heading_rad)

        x_y = np.array([[map_x],
                         [map_y]])
        transform_matrix = np.array([[cos(theta),-sin(theta)],
                                    [sin(theta),cos(theta)]])

        x_y_real = np.matmul(transform_matrix, x_y)    
        return map(float, x_y_real)

    def draw_bbox(self, frame):

        font = cv2.FONT_HERSHEY_SIMPLEX
        line = cv2.LINE_AA

        text_pos = {
                    "green": (50,camera["shape"][1] - 50),  # left
                    "red": (camera["shape"][0] - 250,camera["shape"][1] - 50), # right
                    "pink": (200,camera["shape"][1] - 50)   # middle
                    }

        zone = self.zone
        if self.have_sign:
            frame = cv2.rectangle(frame, (self.x1+zone[0], self.y1+zone[1]), (self.x2+zone[0], self.y2+zone[1]), self.bbox_colour, 1)
            frame = cv2.putText(frame, f"({self.map_x:.2f}, {self.map_y:.2f})",
                                    text_pos[self.type], font, 1, self.bbox_colour, 1, line) 

        return frame
