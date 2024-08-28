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
        self.bus.write_byte_data(self.ADDRESS, self.MODE, 0x00)  # Continuous measurement m

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

    def getAngle(self, calibrate):

        x = self.read_raw_data(self.X_MSB)
        y = self.read_raw_data(self.Y_MSB)
        z = self.read_raw_data(self.Z_MSB)

        heading = round(self.compute_heading(x,y))
        self.heading = heading 

        if 360 >= heading >= self.startPos: 
            angle = heading - self.startPos
        elif 0 <= heading < self.startPos:
            angle = heading + (360 - self.startPos)        

        return angle
        
    def calibrate(self, signal):
        if signal == 1:
            self.startPos = self.heading
