from math import *
import cv2

class Line:

    def __init__(self, line_params, sign_zone=None):

        self.lower = line_params["lower"]
        self.upper = line_params["upper"]
        self.colour = line_params["colour bgr"] # colour for displaying line on image
        self.have_second_range = True
        self.have_third_range = True

        self.max_angle = -999
        self.max_line = None 
        self.min_angle = -999
        self.min_line = None
        self.have_line = False
        # self.line = None
        self.lines = []

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

        if sign_zone is not None:
            self.zone = sign_zone

    def detect_line(self,frame):


        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        self.mask = cv2.inRange(hsv_frame, self.lower, self.upper)

        if self.have_second_range:
            mask2 = cv2.inRange(hsv_frame, self.lower2, self.upper2)
            self.mask = cv2.bitwise_or(mask2, cv2.inRange(hsv_frame, self.lower2, self.upper2))
        
        if self.have_third_range:
            mask3 = cv2.inRange(hsv_frame, self.lower3, self.upper3)
            self.mask = cv2.bitwise_or(mask3, cv2.inRange(hsv_frame, self.lower3, self.upper3))

        self.blur = cv2.GaussianBlur(self.mask, (5, 5), 0)
        self.edges = cv2.Canny(self.mask, 50, 150)
        lines = cv2.HoughLinesP(self.edges, 1, pi/180, threshold=20, minLineLength=50, maxLineGap=20)

        
        if lines is not None:
            maxAng = -999
            minAng = 999
            maxLine = None
            for line in lines:
                x1, y1, x2, y2 = line[0]

                delta_y = y2-y1
                delta_x = x2-x1 if x2-x1 != 0 else 0.01
                
                angle = degrees(abs(atan2((delta_y),(delta_x))))

                if angle > maxAng:
                    maxAng = angle
                    maxLine = (x1, y1, x2, y2)
                
                if angle < minAng:
                    minAng = angle
                    minLine = (x1, y1, x2, y2)
                
            self.max_angle = maxAng
            self.max_line = maxLine
            self.min_angle = minAng
            self.min_line = minLine
            # self.y1 = maxLine[1]
            self.have_line = True
            return self.have_line
        else:
            self.have_line = False
            return self.have_line

    def draw_line(self, frame):
        zone = self.zone
        if self.have_line:
            x1, y1, x2, y2 = self.min_line  # Extract endpoints of the line
            cv2.line(frame, (x1+zone[0], y1+zone[1]), (x2+zone[0], y2+zone[1]), self.colour, 2)  # Draw line
            x1, y1, x2, y2 = self.max_line  # Extract endpoints of the line
            cv2.line(frame, (x1+zone[0], y1+zone[1]), (x2+zone[0], y2+zone[1]), self.colour, 2)  # Draw line
        return frame
    