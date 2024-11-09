import VL53L1X
import time

# Initialize the ToF sensor
tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
tof.open()
tof.start_ranging(1)  # 1 = Short Range, 2 = Medium Range, 3 = Long Range

try:
    while True:
        distance_mm = tof.get_distance()  # Get the distance in mm
        print(f"Distance: {distance_mm} mm")
        # time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopping measurement...")

finally:
    tof.stop_ranging()