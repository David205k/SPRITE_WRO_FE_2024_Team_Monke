import cv2

class TrafficSign:
    """
    Class for the traffic signs
        
    Methods
    -------
    DetectColour()
    PrintLargestBoundingBox()
    """

    def __init__(self, frame, boxColor: tuple, lower, upper, lower2=None, upper2=None, minWidth=200, minHeight=300):
        """
        Parameters
        ----------
        frame: 2d array
            A frame of the video stream
        boxColor: tuple
            Bounding BOx Colour
        lower: tuple
            Lower bound for traffic sign HSV range (h,s,v)
        upper:
            Upper bound for traffic sign HSV (h,s,v)
        lower2:
            Lower bound for 2nd traffic sign HSV range (h,s,v)
        upper2:
            upper bound for 2nd traffic sign HSV range (h,s,v)
        minWidth: int
            Minimum width of traffic sign bounding box (pixels)
        minHeight: int
            Minimum height of traffic sign bouding box (pixels)
        """
        
        self.boxColor = boxColor
        self.frame = frame
        self.minWidth = minWidth
        self.minHeight = minHeight
        self.dist = 0
        
        self.hsvImage = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)   # Convert frame from BGR to HSV
        
        # Create mask for traffic sign hsv range
        self.mask = cv2.inRange(self.hsvImage, lower, upper) 

        # Include 2 range in traffic sign mask
        if lower2 != None and upper2 != None:
            mask2 = cv2.inRange(self.hsvImage, lower2, upper2)  # create mask for second range
            self.mask = cv2.bitwise_or(self.mask, mask2)   # combine with original mask
    
    def detectClear(self) -> tuple:
        """
        Check if there are no traffic signs (of the specified hsv range) ahead 
        """

        contours, _ = cv2.findContours(self.mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        largest_area = 0
        largest_bbox = None

        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            area = w * h

            if area > largest_area and w > 5 and h > 5:
                largest_area = area
                largest_bbox = (x, y, w, h)

        return largest_bbox

    def printLargestBbox(self) :
        """
        Print largest bounding box detected on to the frame.

        Returns:
            - the image frame with the bounding boxes drawn on it
            - largest bounding box found
        """
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

            self.dist = int(self.F_LENGTH*self.ACT_HEIGHT/h)

        return self.frame, largest_bbox