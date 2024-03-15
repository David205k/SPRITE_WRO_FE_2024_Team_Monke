import cv2
from PIL import Image

class trafficSign:

    minWidth = 30
    minHeight = 30

    def __init__(self, bgr, lowerLimit, upperLimit, minDist):
        self.bgr = bgr
        self.height, self.width = 0, 0
        self.minDist = minDist
        self.dist = 0

        mask = cv2.inRange(hsvImage, lowerLimit, upperLimit) #create an image with only the colour inside
        mask_ = Image.fromarray(mask) #convert to Image object so that we can use the library to get bounding box
        self.bbox = mask_.getbbox() #get bounding box


    def printBbox(self):
        if self.bbox is not None:
            x1, y1, x2, y2 = self.bbox

            self.height, self.width = abs(y2-y1), abs(x2-x1)

            if self.height > trafficSign.minHeight and self.width > trafficSign.minWidth:
                self.X, self.Y = (x1+x2)/2, (y1+y2)/2 #get X,Y of Bbox middle

                cv2.rectangle(frame, (x1, y1), (x2, y2), self.bgr, 5) #print bounding box onto frame

                #print("distance = ", (frame.shape[0]/self.height)*self.minDist) #print distance of object to camera
                self.dist = int((frame.shape[0]/self.height)*self.minDist)


capture = cv2.VideoCapture(0) #get video from main camera | 0==> cam 1, 1==> cam2, ...


while True:

    global frame
    global hsvImage

    isTrue, frame = capture.read() #get video frames
    hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)   #convert image from bgr to hsv

    #create trafficSign objects
    green = trafficSign((0,255,0), (40, 100, 100), (70, 255, 255), 8)
    red = trafficSign((0,0,255), (0, 200, 130), (5, 255, 255), 8)

    #print bounding box for trafficSign on the video frames
    green.printBbox()
    red.printBbox()


    #shape[1] ==> width = 640,   shape[0] ==> height = 480
    cv2.line(frame, (frame.shape[1]//2, 0), (frame.shape[1]//2, frame.shape[0]), (0, 255, 255), thickness = 3) #centre line
    cv2.putText(frame, "Dist: " + str(green.dist), (500, 440), cv2.FONT_HERSHEY_TRIPLEX, 1.0, (0, 255, 0), 1)
    cv2.putText(frame, "Dist: " + str(red.dist), (20, 440), cv2.FONT_HERSHEY_TRIPLEX, 1.0, (0, 0, 255), 1)

    cv2.imshow('Camera', frame)  #show video frames

    if cv2.waitKey(1) & 0xFF == ord('q'): #break out of loop if 'q' is pressed
        break

capture.release()
cv2.destroyAllWindows()