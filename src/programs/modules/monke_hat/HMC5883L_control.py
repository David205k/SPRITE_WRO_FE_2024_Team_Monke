"""
Changing i2c frequency for RPI:
- Add this to/boot/firmware/config.txt, "dtparam=i2c_arm=on,i2c_arm_baudrate=400000"
- Reboot pi
"""

import smbus
import math

class Compass:
    """
    Class for utilising the HMC5883L magnetometer module

    Methods
    -------
    read_raw_data(addr)
    compute_heading(x,y)
    get_angle(relative=True)
    apply_calibration(raw_x, raw_y, raw_z)
    set_home(signal=True)
    """

    # HMC5883L register addresses
    CONFIG_A = 0x00
    CONFIG_B = 0x01
    MODE = 0x02
    X_MSB = 0x03
    Z_MSB = 0x05
    Y_MSB = 0x07

    # area specific alterations
    DECLINATION_ANGLE = 0.22
    # X_OFFSET, Y_OFFSET, Z_OFFSET = -28.0, -138.0, 0
    # X_SCALE, Y_SCALE, Z_SCALE = 511.0, 498.0, 1
    X_OFFSET, Y_OFFSET, Z_OFFSET = -62.5, -257.0, 0.0
    X_SCALE, Y_SCALE, Z_SCALE = 485.5, 480.0, 2.0
    bus = None

    def __init__(self, addr=0x1E):
        """
        Parameters
        ----------

        addr: int
            i2c Address of HMC5583L module. (Default is 0x1E) 
            Fake China clones have a different address. 
        """

        self.bus = smbus.SMBus(1)

        self.ADDRESS = addr

        self.bus.write_byte_data(self.ADDRESS, self.CONFIG_A, 0x70)  # Set to 8 samples @ 15Hz
        self.bus.write_byte_data(self.ADDRESS, self.CONFIG_B, 0x20)  # 1.3 gain LSb / Gauss 1090 (default)
        self.bus.write_byte_data(self.ADDRESS, self.MODE, 0x00)  # Continuous measurement mode

        self.startPos = 0

    def read_raw_data(self, addr) -> int:
        """
        Reads 16 bit value from HMC5583L registers

        Parameters
        ----------

        addr: int
            HMC5883L register addresses
        """

        high = self.bus.read_byte_data(self.ADDRESS, addr)
        low = self.bus.read_byte_data(self.ADDRESS, addr+1)
        
        # Combine them to get a 16-bit value
        value = (high << 8) + low
        if value > 32768:  # Adjust for 2's complement
            value = value - 65536
        return value
    
    def compute_heading(self, x, y):
        """
        Calculate heading in radians

        Parameters
        ----------
        x: int
            16 bit value from x register
        y: int
            16 bit value from y register
        """

        heading_rad = math.atan2(y, x)
        
        heading_rad += self.DECLINATION_ANGLE  # Adjust for declination angle (e.g. 0.22 for ~13 degrees)
        
        # Correct for when signs are reversed.
        if heading_rad < 0:
            heading_rad += 2 * math.pi
    
        # Check for wrap due to addition of declination.
        if heading_rad > 2 * math.pi:
            heading_rad -= 2 * math.pi

        return math.degrees(heading_rad)

    def get_angle(self, relative=True) -> int:
        """
        Get heading from HMC5883L compass. 0-360 degrees

        Read the raw values from each axes, apply offset values and calculate heading.
        Parameters
        ----------
        relative: bool
            True: Return the heading of the compass relative to the home position.
            False: Return the true heading
        """
        # read compass angle (catch exception when compass is not connected properly)
        x = self.read_raw_data(self.X_MSB)
        y = self.read_raw_data(self.Y_MSB)
        z = self.read_raw_data(self.Z_MSB)

        x, y, z = self.apply_calibration(x, y, z)

        heading = round(self.compute_heading(x,y))

        if relative: # get angle relative to start position
            if 360 >= heading >= self.startPos: 
                angle = heading - self.startPos
            elif 0 <= heading < self.startPos:
                angle = heading + (360 - self.startPos)        
        else:
            angle = heading

        return angle
        
    def apply_calibration(self, raw_x, raw_y, raw_z):
        """
        Apply the offset and scale from calibration to raw readings
        """

        corrected_x = (raw_x - self.X_OFFSET) / self.X_SCALE
        corrected_z = (raw_z - self.Z_OFFSET) / self.Z_SCALE
        corrected_y = (raw_y - self.Y_OFFSET) / self.Y_SCALE
        
        return corrected_x, corrected_y, corrected_z

    def set_home(self, signal=True):
        """
        Set the reference position to read the compass angle from.
        """

        if signal == True:
            self.startPos = self.get_angle(relative=False)
