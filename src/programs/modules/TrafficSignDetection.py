# modules for camera vision 
import cv2
from PIL import Image      

class TrafficSign:

    minWidth = 300
    minHeight = 400

    def __init__(self, frame, bgr, lowerLimit, upperLimit, minDist):
        self.bgr = bgr
        self.minDist = minDist
        self.frame = frame

        self.hsvImage = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)   # Convert image from BGR to HSV
        self.mask = cv2.inRange(self.hsvImage, lowerLimit, upperLimit)  # Create mask for the color

    def printLargestBbox(self):
        contours, _ = cv2.findContours(self.mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        largest_area = 0
        largest_bbox = None

        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            area = w * h

            if area > largest_area and w > TrafficSign.minWidth and h > TrafficSign.minHeight:
                largest_area = area
                largest_bbox = (x, y, w, h)

        if largest_bbox:
            x, y, w, h = largest_bbox
            cv2.rectangle(self.frame, (x, y), (x + w, y + h), self.bgr, 5)

        return self.frame, largest_bbox

# Release the capture and close any OpenCV windows
cap.release()
cv2.destroyAllWindows()