from math import *
import cv2

class Line:

    def __init__(self, line_params, sign_zone=None):

        self.lower = line_params["lower"]
        self.upper = line_params["upper"]
        self.colour = line_params["colour bgr"] # colour for displaying line on image
        self.have_second_range = True

        self.angle = -999
        self.have_line = False
        self.line = None

        try:
            self.lower2 = line_params["lower2"]
            self.upper2 = line_params["upper2"]  
        except KeyError:
            self.have_second_range = False

        if sign_zone is not None:
            self.zone = sign_zone

    def detect_line(self,frame):

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        self.mask = cv2.inRange(hsv_frame, self.lower, self.upper)

        if self.have_second_range:
            mask2 = cv2.inRange(hsv_frame, self.lower2, self.upper2)
            self.mask = cv2.bitwise_or(mask2)
        
        lines = cv2.HoughLinesP(self.mask, 1, pi/180, threshold=35, minLineLength=200, maxLineGap=5)

        if lines is not None:
            maxAng = -999
            maxLine = None
            for line in lines:
                x1, y1, x2, y2 = line[0]
                try:
                    angle = degrees(abs(atan((y2-y1)/(x2-x1))))
                except ZeroDivisionError:
                    angle = 0
                if angle > maxAng:
                    maxAng = angle
                    maxLine = (x1, y1, x2, y2)
                
            self.angle = maxAng
            self.line = maxLine
            self.y1 = maxLine[1]
            self.have_line = True
            return self.have_line
        else:
            self.have_line = False
            return self.have_line

    def draw_line(self, frame):
        zone = self.zone
        if self.have_line:
            x1, y1, x2, y2 = self.line  # Extract endpoints of the line
            cv2.line(frame, (x1+zone[0], y1+zone[1]), (x2+zone[0], y2+zone[1]), self.colour, 2)  # Draw line
        return frame
    