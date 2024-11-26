import cv2
import numpy as np

def bgr_to_hsv(bgr):
    """
    Convert a BGR color tuple to HSV.
    
    Parameters:
        bgr (tuple): A tuple of (B, G, R) values.
        
    Returns:
        tuple: A tuple of (H, S, V) values.
    """
    # Create a 1x1 pixel image with the BGR color
    color_bgr = np.uint8([[bgr]])
    # Convert BGR to HSV
    color_hsv = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2HSV)
    # Return HSV values as a tuple
    return tuple(color_hsv[0][0])

# Example usage
while True:

    r,g,b = input("Input RGB:").split()
    bgr_color = int(b), int(g), int(r)
    hsv_color = bgr_to_hsv(bgr_color)
    print("HSV color:", hsv_color)