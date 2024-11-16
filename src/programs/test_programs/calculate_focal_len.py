import cv2
from picamera2 import Picamera2
import threading
import time
from math import *

picam2 = Picamera2()
picam2.preview_configuration.main.size=(1920,1000)
picam2.preview_configuration.main.format = 'RGB888'
picam2.start()

CAM_SHAPE = (640, 480)

def read_cam():
    while True:

        im = picam2.capture_array()

        im = cv2.flip(im, 0) # Flip vertically
        im = cv2.flip(im, 1) # Flip horizontally
        
        im = cv2.resize(im, CAM_SHAPE)

        cv2.imshow('preview', im)

        if cv2.waitKey(1) == ord('q'):
            break

    picam2.stop()
    cv2.destroyAllWindows()

def get_focal_len():
    user_continue = 'y'
    number_of_samples = 0
    focal_length_sum = 0
    time.sleep(2)
    while user_continue == 'y':
        act_length = float(input("Enter Actual Distance of Object to Camera in cm: "))
        act_height = float(input("Enter Actual Height of Object in cm: "))
        top_pixel, bottom_pixel = input("Enter Y coordinate of pixel of top and bottom of object (top, bottom): ").split()
        
        top_pixel, bottom_pixel = float(top_pixel), float(bottom_pixel)
           
        user_continue = input("Do you want to coninue? (y for yes and n for no): ")

        focal_length_sum += (bottom_pixel-top_pixel)/act_height * act_length

        number_of_samples+=1
    
    focal_length_average = focal_length_sum / number_of_samples

    print(f"Average focal length (pixels): {focal_length_average}")

try: 
    background_thread = threading.Thread(target=read_cam, daemon=True)
    background_thread.start()
    try:
        get_focal_len()
    except Exception as E:
        print("Error occured: ", E)
except Exception as E:
    print("Error occured: ", E)
finally:
    background_thread.join() 
    picam2.stop()
    cv2.destroyAllWindows()
