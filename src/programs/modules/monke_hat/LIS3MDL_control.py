# Check i2c device: i2c detect -y <bus number>
# LIS3MDL Datasheet: https://www.adafruit.com/product/4479 
# address: 0x1c

from math import atan2, degrees
import board
import adafruit_lis3mdl
import csv

class Compass:

    # compass offset and scale values (specific to envrionment and module)

    X_OFFSET, Y_OFFSET, Z_OFFSET = -1239.5, -670.5, -1536.5
    X_SCALE, Y_SCALE, Z_SCALE = 3362.5, 3182.5, 762.5

    def __init__(self):

        i2c = board.I2C()  # uses board.SCL and board.SDA

        self.lis3mdl = adafruit_lis3mdl.LIS3MDL(i2c)

        self.startPos = 0

    def calculate_offsets(self, x_readings, y_readings, z_readings):
        x_offset = (max(x_readings) + min(x_readings)) / 2
        y_offset = (max(y_readings) + min(y_readings)) / 2
        z_offset = (max(z_readings) + min(z_readings)) / 2
        
        x_scale = (max(x_readings) - min(x_readings)) / 2
        y_scale = (max(y_readings) - min(y_readings)) / 2
        z_scale = (max(z_readings) - min(z_readings)) / 2
        
        print(f"X_OFFSET, Y_OFFSET, Z_OFFSET = {x_offset}, {y_offset}, {z_offset}")
        print(f"X_SCALE, Y_SCALE, Z_SCALE = {x_scale}, {y_scale}, {z_scale}")

        return x_offset, y_offset, z_offset, x_scale, y_scale, z_scale

    # Function to save calibration data to CSV
    def save_to_csv(self, x_readings, y_readings, z_readings, filename="calibration_data.csv"):
        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            
            writer.writerow(["X", "Y", "Z"]) # Write headers
            
            for x, y, z in zip(x_readings, y_readings, z_readings): # Write data
                writer.writerow([x, y, z])

        print("Storing calibration data in Excel...")
        print(f"Calibration data saved to {filename}")

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
                raw_magnet_x, raw_magnet_y, _ = self.lis3mdl._raw_mag_data
                # magnet_x, magnet_y, _ = self.lis3mdl.magnetic
                magnet_x, magnet_y = self.apply_calibration(raw_magnet_x, raw_magnet_y)
            except OSError:
                print("Unable to connect to compass.")
                continue
            print("Connection to compass restored.")
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

