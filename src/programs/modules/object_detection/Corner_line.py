from math import *
import cv2

class Line:

    def __init__(self, line_params, sign_zone=None):

        self.lower = line_params["lower"]   # hsv range lower bound
        self.upper = line_params["upper"]   # hsv range upper bound
        self.colour = line_params["colour bgr"] # colour for displaying line on image
        self.have_second_range = True       
        self.have_third_range = True

        # self.max_angle = -999
        # self.max_line = None 
        # self.min_angle = -999
        # self.min_line = None
        # self.have_line = False
        self.lines = {}
        self.angles = {}


        try:
            self.lower2 = line_params["lower2"]
            self.upper2 = line_params["upper2"]  
        except KeyError:
            self.have_second_range = False

        try:
            self.lower3 = line_params["lower3"]
            self.upper3 = line_params["upper3"]  
        except KeyError:
            self.have_third_range = False

        self.roi = sign_zone

    def detect_line(self,frame):

        if self.roi: # crop out region of interest if there its specified
            cropped_out_frame = frame[self.roi[1]:self.roi[1]+self.roi[3], self.roi[0]:self.roi[0]+self.roi[2]]
        else:
            cropped_out_frame = frame

        hsv_frame = cv2.cvtColor(cropped_out_frame, cv2.COLOR_BGR2HSV)
        self.mask = cv2.inRange(hsv_frame, self.lower, self.upper)

        # apply second and third hsv range to get mask
        if self.have_second_range:
            mask2 = cv2.inRange(hsv_frame, self.lower2, self.upper2)
            self.mask = cv2.bitwise_or(mask2, cv2.inRange(hsv_frame, self.lower2, self.upper2))
        if self.have_third_range:
            mask3 = cv2.inRange(hsv_frame, self.lower3, self.upper3)
            self.mask = cv2.bitwise_or(mask3, cv2.inRange(hsv_frame, self.lower3, self.upper3))

        self.blur = cv2.GaussianBlur(self.mask, (5, 5), 0) # blur the mask to reduce noise
        self.edges = cv2.Canny(self.mask, 50, 150) # get edges from blurred pic
        lines = cv2.HoughLinesP(self.edges, 1, pi/180, threshold=20, minLineLength=50, maxLineGap=20) # detect lines
        
        if lines is not None:
            self.angles["max"] = -999 
            self.angles["min"]= 999 
            for line in lines:
                x1, y1, x2, y2 = line[0]

                delta_y = y2-y1
                delta_x = x2-x1 if x2-x1 != 0 else 0.01 # account for zero division
                
                angle = degrees((atan2((delta_y),(delta_x))))

                if angle > self.angles["max"]:
                    self.angles["max"] = angle
                    self.lines["max"] = (x1, y1, x2, y2)
                
                if angle < self.angles["min"]:
                    self.angles["min"] = angle
                    self.lines["min"] = (x1, y1, x2, y2)
            
            self.have_line = True
            return self.have_line
        else:
            self.have_line = False
            return self.have_line

    def draw_line(self, camera_frame):

        roi = self.roi

        if roi:
            dx, dy = roi[0], roi[1]
        else:
            dx, dy = 0, 0
            
        for line in self.lines.values():
            x1, y1, x2, y2 = line  # Extract endpoints of the line
            cv2.line(camera_frame, (x1+dx, y1+dy), (x2+dx, y2+dy), self.colour, 2)  # Draw line
            
        
        return camera_frame
    