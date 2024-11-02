import cv2
from picamera2 import Picamera2

picam2 = Picamera2()
picam2.preview_configuration.main.size=(1920,1000)
picam2.preview_configuration.main.format = 'RGB888'
picam2.start()

def main():
    while True:
        frame = picam2.capture_array()

        frame = cv2.flip(frame, 0) # Flip vertically
        frame = cv2.flip(frame, 1) # Flip horizontally

        cv2.imshow('Camera', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'): #break out of loop if 'q' is pressed
            cv2.destroyAllWindows()
            break

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        picam2.stop()
        cv2.destroyAllWindows() 