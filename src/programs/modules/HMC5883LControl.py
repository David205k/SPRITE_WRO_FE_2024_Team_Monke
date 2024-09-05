import smbus
import math

class compass:

    # HMC5883L register addresses
    ADDRESS = 0x1E
    CONFIG_A = 0x00
    CONFIG_B = 0x01
    MODE = 0x02
    X_MSB = 0x03
    Z_MSB = 0x05
    Y_MSB = 0x07

    startPos = 0
    heading = 0
    angle = 0

    bus = None

    def __init__(self, addr):

        self.bus = smbus.SMBus(1)

        self.ADDRESS = addr

        self.bus.write_byte_data(self.ADDRESS, self.CONFIG_A, 0x70)  # Set to 8 samples @ 15Hz
        self.bus.write_byte_data(self.ADDRESS, self.CONFIG_B, 0x20)  # 1.3 gain LSb / Gauss 1090 (default)
        self.bus.write_byte_data(self.ADDRESS, self.MODE, 0x00)  # Continuous measurement mode

    def read_raw_data(self, addr):
        # Read raw 16-bit value
        high = self.bus.read_byte_data(self.ADDRESS, addr)
        low = self.bus.read_byte_data(self.ADDRESS, addr+1)
        
        # Combine them to get a 16-bit value
        value = (high << 8) + low
        if value > 32768:  # Adjust for 2's complement
            value = value - 65536
        return value
    
    def compute_heading(self, x, y):
        # Calculate heading in radians
        heading_rad = math.atan2(y, x)
        
        # Adjust for declination angle (e.g. 0.22 for ~13 degrees)
        declination_angle = 0.22
        heading_rad += declination_angle
        
        # Correct for when signs are reversed.
        if heading_rad < 0:
            heading_rad += 2 * math.pi
    
        # Check for wrap due to addition of declination.
        if heading_rad > 2 * math.pi:
            heading_rad -= 2 * math.pi

            
        # Convert radians to degrees for readability.
        heading_deg = heading_rad * (180.0 / math.pi)
        
        return heading_deg

    def getAngle(self):

        # x_offset, y_offset, z_offset = -22.5, -132.0, 0
        # x_scale, y_scale, z_scale = 446.5, 452.0, 1
        x_offset, y_offset, z_offset = -18.5, -119.0, 0
        x_scale, y_scale, z_scale = 384.5, 392.0, 1

        x = self.read_raw_data(self.X_MSB)
        y = self.read_raw_data(self.Y_MSB)
        z = self.read_raw_data(self.Z_MSB)

        x, y, z = self.apply_calibration(x, y, z, x_offset, y_offset, z_offset, x_scale, y_scale, z_scale)

        heading = round(self.compute_heading(x,y))
        self.heading = heading 

        if 360 >= heading >= self.startPos: 
            angle = heading - self.startPos
        elif 0 <= heading < self.startPos:
            angle = heading + (360 - self.startPos)        

        return angle
        
    def apply_calibration(self, raw_x, raw_y, raw_z, x_offset, y_offset, z_offset, x_scale, y_scale, z_scale):
        # Apply the offset and scale to raw readings
        corrected_x = (raw_x - x_offset) / x_scale
        corrected_y = (raw_y - y_offset) / y_scale
        corrected_z = (raw_z - z_offset) / z_scale
        
        return corrected_x, corrected_y, corrected_z

    def setHome(self, signal=True):
        if signal == True:
            # x_offset, y_offset, z_offset = -18.5, -119.0, 0
            # x_scale, y_scale, z_scale = 384.5, 392.0, 1

            x_offset, y_offset, z_offset = -41, -113.5, 0
            x_scale, y_scale, z_scale = 621, 629.5, 1

            x = self.read_raw_data(self.X_MSB)
            y = self.read_raw_data(self.Y_MSB)
            z = self.read_raw_data(self.Z_MSB)

            x, y, z = self.apply_calibration(x, y, z, x_offset, y_offset, z_offset, x_scale, y_scale, z_scale)

            heading = round(self.compute_heading(x,y))

            self.startPos = heading
