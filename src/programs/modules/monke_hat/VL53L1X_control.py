import math
from RPi import GPIO
from PiicoDev_VL53L1X import PiicoDev_VL53L1X

class tof_manager: 

    def __init__(self, tof_sensors_params):

        # turn off all tof sensors first
        self.tof_sensors_params = tof_sensors_params

        for sensor in tof_sensors_params:
            GPIO.setup(sensor["x shut"], GPIO.OUT)
            GPIO.output(sensor["x shut"], GPIO.LOW)
        
        # turn on sensor and change their i2c address
        start_addr = 0x30
        tof_sensors = []
        for i, sensor in enumerate(tof_sensors_params):
            GPIO.output(sensor["x shut"], GPIO.HIGH)
            tof_sensors.append(PiicoDev_VL53L1X( bus=1, sda=27, scl=28, freq = 400_000 ))
            tof_sensors[i].change_addr(start_addr+1+i)

        self.tof_sensors = tof_sensors

    def read_sensors(self):
        for sensor in self.tof_sensors:
            dist = sensor.read() // 10

            # reinitalise sensors if sensors disconnect 
            if math.isnan(dist):
                print("Re-initialising tofs")
                try:
                    self.__init__(self.tof_sensors_params)
                except OSError:
                    pass

            yield dist