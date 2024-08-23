import time
import math
import smbus
from Adafruit_HMC5883L import HMC5883L

# Initialize the HMC5883L sensor
mag = HMC5883L()

# Variables
input_char = None
ang = 0
startAng = 0
maxAng = -9999
minAng = 9999

def displaySensorDetails():
    print("------------------------------------")
    print("Sensor:       HMC5883L")
    print("Driver Ver:   1.0")
    print("Unique ID:    12345")
    print("Max Value:    +/- 8.1 Gauss")
    print("Min Value:    +/- 0.88 Gauss")
    print("Resolution:   0.16 mGa")
    print("------------------------------------")
    print("")
    time.sleep(0.5)

def setup():
    print("HMC5883 Magnetometer Test")
    print("")

    # Display some basic information on this sensor
    displaySensorDetails()

def loop():
    global ang, startAng, maxAng, minAng, input_char

    # Check if input is available (simulating Arduino's Serial.read)
    input_char = input("Enter 's' to set start angle: ")

    # Get the magnetic reading from the sensor
    x, y, z = mag.read()
    
    # Calculate heading
    heading = math.atan2(y, x)
    
    # Correct for negative values
    if heading < 0:
        heading += 2 * math.pi
    
    # Convert radians to degrees for readability
    headingDegrees = heading * 180 / math.pi
    print("Heading (degrees): {:.2f}".format(headingDegrees))

    if input_char == 's':
        startAng = headingDegrees

    if 360 >= headingDegrees >= startAng:
        ang = headingDegrees - startAng
    elif 0 <= headingDegrees < startAng:
        ang = headingDegrees + 360 - startAng

    # maxAng = max(ang, maxAng)
    # minAng = min(ang, minAng)

    print("ang: {:.2f}".format(ang))
    # print("Max: {:.2f} Min: {:.2f}".format(maxAng, minAng))

    time.sleep(0.5)

if __name__ == "__main__":
    setup()
    while True:
        loop()
