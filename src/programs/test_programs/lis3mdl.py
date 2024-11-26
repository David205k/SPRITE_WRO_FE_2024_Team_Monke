import smbus
import time

class LIS3MDL:
    DEFAULT_ADDRESS = 0x1C

    CTRL_REG1 = 0x20
    CTRL_REG2 = 0x21
    CTRL_REG3 = 0x22
    OUT_X_L = 0x28
    OUT_X_H = 0x29
    OUT_Y_L = 0x2A
    OUT_Y_H = 0x2B
    OUT_Z_L = 0x2C
    OUT_Z_H = 0x2D

    def __init__(self, bus_id=1, address=DEFAULT_ADDRESS):
        self.bus = smbus.SMBus(bus_id)
        self.address = address
        self._init_sensor()

    def _init_sensor(self):
        self.bus.write_byte_data(self.address, self.CTRL_REG1, 0x70)
        self.bus.write_byte_data(self.address, self.CTRL_REG2, 0x00)
        self.bus.write_byte_data(self.address, self.CTRL_REG3, 0x00)
        print(f"LIS3MDL initialized at 0x{self.address:02X}")

    def _read_raw_data(self, low_reg, high_reg):
        low = self.bus.read_byte_data(self.address, low_reg)
        high = self.bus.read_byte_data(self.address, high_reg)
        value = (high << 8) | low
        return value - 65536 if value > 32767 else value

    def read_magnetic(self):
        x = self._read_raw_data(self.OUT_X_L, self.OUT_X_H)
        y = self._read_raw_data(self.OUT_Y_L, self.OUT_Y_H)
        z = self._read_raw_data(self.OUT_Z_L, self.OUT_Z_H)
        return x, y, z

    def calibrate(self, duration=10):
        """
        Perform calibration by collecting raw data over a specified duration.
        Returns offset and scale values for X, Y, and Z axes.

        :param duration: Duration to collect data in seconds.
        :return: Tuple of offsets and scales ((x_offset, y_offset, z_offset), (x_scale, y_scale, z_scale)).
        """
        print("Starting calibration. Move the sensor in all orientations.")
        min_vals = [float('inf')] * 3
        max_vals = [-float('inf')] * 3

        start_time = time.time()
        while time.time() - start_time < duration:
            x, y, z = self.read_magnetic()
            min_vals[0] = min(min_vals[0], x)
            max_vals[0] = max(max_vals[0], x)
            min_vals[1] = min(min_vals[1], y)
            max_vals[1] = max(max_vals[1], y)
            min_vals[2] = min(min_vals[2], z)
            max_vals[2] = max(max_vals[2], z)
            time.sleep(0.05)  # Collect readings every 50 ms

        # Calculate offsets and scales
        offsets = [(min_vals[i] + max_vals[i]) / 2 for i in range(3)]
        scales = [(max_vals[i] - min_vals[i]) / 2 for i in range(3)]

        print("Calibration complete.")
        print(f"X_OFFSET, Y_OFFSET, Z_OFFSET = {offsets[0]}, {offsets[1]}, {offsets[2]}")
        print(f"X_SCALE, Y_SCALE, Z_SCALE = {scales[0]}, {scales[1]}, {scales[2]}")

        return offsets, scales

    def close(self):
        self.bus.close()

# Usage Example
if __name__ == "__main__":
    try:
        compass = LIS3MDL()

        # Calibrate the sensor
        offsets, scales = compass.calibrate(duration=30)

        # Read and apply calibration
        while True:
            x, y, z = compass.read_magnetic()

            # Apply calibration
            x_corrected = (x - offsets[0]) / scales[0]
            y_corrected = (y - offsets[1]) / scales[1]
            z_corrected = (z - offsets[2]) / scales[2]

            print(f"Corrected magnetic field: X={x_corrected:.2f}, Y={y_corrected:.2f}, Z={z_corrected:.2f}")
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nExiting program.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        compass.close()
