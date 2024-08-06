# Import necessary libraries for I2C communication, time management, and mathematical calculations
import smbus
import time
import math

# Define a class for Magnetometer control
class Magnetometer:
    def __init__(self):
        self.bus = smbus.SMBus(1)  # Initialize the I2C bus
        self.address = 0x1e        # Magnetometer I2C address
        self.setup()               # Call the setup method

    # Setup the magnetometer with initial configurations
    def setup(self):
        self.bus.write_byte_data(self.address, 0x00, 0x70)  # Set to 8 samples @ 15Hz
        self.bus.write_byte_data(self.address, 0x01, 0xa0)  # Set gain
        self.bus.write_byte_data(self.address, 0x02, 0x00)  # Set continuous measurement mode

    # Read raw data from the specified register
    def read_raw_data(self, addr):
        high = self.bus.read_byte_data(self.address, addr)
        low = self.bus.read_byte_data(self.address, addr + 1)
        value = ((high << 8) | low)
        if value > 32768:
            value = value - 65536
        return value

    # Get the heading based on magnetometer readings
    def get_heading(self):
        x = self.read_raw_data(0x03)
        y = self.read_raw_data(0x07)
        z = self.read_raw_data(0x05)

        heading = math.atan2(y, x) + 0.45  # Adding offset for true north
        if heading < 0:
            heading += 2 * math.pi
        if heading > 2 * math.pi:
            heading -= 2 * math.pi
        return math.degrees(heading)

    # Check if a lap is complete based on the heading
    def is_lap_complete(self, start_heading):
        current_heading = self.get_heading()
        return abs(current_heading - start_heading) < 10  # 10 degrees threshold
