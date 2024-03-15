import cv2
import numpy as np

#Create blank image
blank = np.zeros((500, 500, 3), dtype = 'uint8') #uint8 ==> image datatype
cv2.imshow('Blank', blank) #show blank image

# #1. paint image a certain colour
# blank[100:300, 300:400] = 0,255,0
# cv2.imshow('Green', blank)

# #2. Draw a rectangle
# cv2.rectangle(blank, (0,0), (blank.shape[1]//2, blank.shape[0]//2), (0, 255, 0), thickness = cv2.FILLED) #(image, pt1, pt2, linecolour, linethickness) 
# cv2.imshow("Rectangle", blank)


# #3. Draw a circle
# cv2.circle(blank, (blank.shape[1]//2, blank.shape[0]//2), 40, (0, 0, 255), thickness = 3)
# cv2.imshow("Circle", blank)

# #4. Draw a line 
# cv2.line(blank, (0, 0), (250, 250), (255, 255, 255), thickness=3)
# cv2.imshow("Line", blank)

#5. Write Text
cv2.putText(blank, 'Hello', (225, 225), cv2.FONT_HERSHEY_TRIPLEX, 1.0, (0, 255, 0), 2)
cv2.imshow("Text", blank)

cv2.waitKey(0)
