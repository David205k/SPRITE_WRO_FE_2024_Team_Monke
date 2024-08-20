import cv2 

#fuction for rescaling video 
def rescaleFrame(frame, scale = 0.75):
    #Works for images, videos, live videos
    width = int(frame.shape[1] * scale)
    height = int(frame.shape[0] * scale)
    dimensions = (width, height)

    return cv2.resize(frame, dimensions, interpolation=cv2.INTER_AREA)


capture = cv2.VideoCapture(0)     #0 ==> webcam, 1 ==> 1st camera, 2 ==> 2nd camera

while True:

    isTrue, frame = capture.read()

    frame_resized = rescaleFrame(frame, scale=0.2)

    cv2.imshow('Video', frame)
    cv2.imshow('Video Resized', frame_resized)

    if cv2.waitKey(20) & 0xFF==ord('d'):
        break

capture.release()
cv2.destroyAllWindows()