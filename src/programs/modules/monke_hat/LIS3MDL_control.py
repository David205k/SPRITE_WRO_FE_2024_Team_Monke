# Check i2c device: i2c detect -y <bus number>
# LIS3MDL Datasheet: https://www.adafruit.com/product/4479 
# address: 0x1c

from math import atan2, degrees
import board
import adafruit_lis3mdl

class compass:

    # compass offset and scale values (specific to envrionment and module)
    X_OFFSET, Y_OFFSET, Z_OFFSET = -23.42882198187664, 24.269219526454254, 6.248173048816135
    X_SCALE, Y_SCALE, Z_SCALE = 45.834551300789244, 46.616486407483194, 12.372113417129494

    def __init__(self):

        i2c = board.I2C()  # uses board.SCL and board.SDA

        self.lis3mdl = adafruit_lis3mdl.LIS3MDL(i2c)

        self.startPos = 0

    def apply_calibration(self, raw_x, raw_y):
        """
        Apply the offset and scale from calibration to raw readings
        """

        corrected_x = (raw_x - self.X_OFFSET) / self.X_SCALE
        corrected_y = (raw_y - self.Y_OFFSET) / self.Y_SCALE
        
        return corrected_x, corrected_y

    def get_angle(self, relative=True):
        while True:
            try:
                magnet_x, magnet_y, _ = self.lis3mdl.magnetic
                magnet_x, magnet_y = self.apply_calibration(magnet_x, magnet_y)
            except OSError:
                print("Unable to connect to compass.")
                continue
            break

        angle = degrees(atan2(magnet_y, magnet_x))
        if angle < 0:
            angle += 360
        
        if relative: # get angle relative to start position
            if 360 >= angle >= self.startPos: 
                angle -= self.startPos
            elif 0 <= angle < self.startPos:
                angle += (360 - self.startPos)        

        return angle

    def set_home(self, signal=True):
        """
        Set the reference position to read the compass angle from.
        """
        if signal == True:
            self.startPos = self.get_angle(relative=False)

