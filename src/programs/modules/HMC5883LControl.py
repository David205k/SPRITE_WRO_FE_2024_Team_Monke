import smbus
import time
import math
import RPi.GPIO as GPIO # use RPi library for controlling GPIO pins

GPIO.setwarnings(False) # turn off warnings for pins (if pins were previously used and not released properly there will be warnings)
GPIO.setmode(GPIO.BOARD) # pin name convention used is pin numbers on board
 
# HMC5883L register addresses
ADDRESS = 0x1E
CONFIG_A = 0x00
CONFIG_B = 0x01
MODE = 0x02
X_MSB = 0x03
Z_MSB = 0x05
Y_MSB = 0x07
 
bus = smbus.SMBus(1)
 
butPin = 16
GPIO.setup(butPin,GPIO.IN)

def setup():
    bus.write_byte_data(ADDRESS, CONFIG_A, 0x70)  # Set to 8 samples @ 15Hz
    bus.write_byte_data(ADDRESS, CONFIG_B, 0x20)  # 1.3 gain LSb / Gauss 1090 (default)
    bus.write_byte_data(ADDRESS, MODE, 0x00)  # Continuous measurement mode
 

def read_raw_data(addr):
    # Read raw 16-bit value
    high = bus.read_byte_data(ADDRESS, addr)
    low = bus.read_byte_data(ADDRESS, addr+1)
    
    # Combine them to get a 16-bit value
    value = (high << 8) + low
    if value > 32768:  # Adjust for 2's complement
        value = value - 65536
    return value
 
def compute_heading(x, y):
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
 
startPos = 0 
def main():
    setup()
    
    global startPos

    while True:
        try:
            x = read_raw_data(X_MSB)
            y = read_raw_data(Y_MSB)
            z = read_raw_data(Z_MSB)
            
            heading = compute_heading(x, y)
            
            # print(f"X: {x} uT, Y: {y} uT, Z: {z} uT, Heading: {heading:.2f}°")
            
            print(GPIO.input(butPin))

            if GPIO.input(butPin) == 1:
                startPos = heading

            if 360 >= heading >= startPos: 
                angle = heading - startPos
            elif 0 <= heading < startPos:
                angle = heading + (360 - startPos)

            print(f"heading: {heading} angle: {angle} startPos: {startPos}")

            time.sleep(0.1)
        except KeyboardInterrupt:
            break
 
if __name__ == "__main__":
    main()