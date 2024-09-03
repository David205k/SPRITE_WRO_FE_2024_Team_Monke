# modules for camera vision 
import cv2
from PIL import Image      
import numpy as np  # Add this line to import NumPy

class TrafficSign:

    def __init__(self, frame, boxColor, lower, upper, minWidth=200, minHeight=300):
        self.boxColor = boxColor
        self.frame = frame
        self.minWidth = minWidth
        self.minHeight = minHeight

        # lowerLimit = cv2.cvtColor(np.uint8([[lowerLimit]]), cv2.COLOR_BGR2HSV)[0][0]
        # upperLimit = cv2.cvtColor(np.uint8([[upperLimit]]), cv2.COLOR_BGR2HSV)[0][0]

        self.hsvImage = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)   # Convert image from BGR to HSV
        self.mask = cv2.inRange(self.hsvImage, lower, upper)  # Create mask for the color
    
    def detectClr(self):
        contours, _ = cv2.findContours(self.mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        largest_area = 0
        largest_bbox = None

        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            area = w * h

            if area > largest_area and w > 10 and h > 10:
                largest_area = area
                largest_bbox = (x, y, w, h)

        return largest_bbox

    def printLargestBbox(self):
        contours, _ = cv2.findContours(self.mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        largest_area = 0
        largest_bbox = None

        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)

            area = w * h

            if area > largest_area and w > self.minWidth and h > self.minHeight:
                self.x = x + w/2
                self.y = y + h/2
                largest_area = area
                largest_bbox = (x, y, w, h)

        if largest_bbox:
            x, y, w, h = largest_bbox
            cv2.rectangle(self.frame, (x, y), (x + w, y + h), self.boxColor, 5)

        return self.frame, largest_bbox
    