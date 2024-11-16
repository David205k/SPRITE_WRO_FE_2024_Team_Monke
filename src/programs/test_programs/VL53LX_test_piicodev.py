# repo: https://github.com/CoreElectronics/CE-PiicoDev-VL53L1X-MicroPython-Module
# module bought from: https://github.com/CoreElectronics/CE-PiicoDev-VL53L1X-MicroPython-Module
# default i2c addr: 0x29
#


from PiicoDev_VL53L1X import PiicoDev_VL53L1X 
import time

from RPi import GPIO

tof1 = PiicoDev_VL53L1X( bus=0, sda=2, scl=3, freq = 400_000)
tof2 = PiicoDev_VL53L1X( bus=1, sda=27, scl=28, freq = 400_000 )
tof2.change_addr(0x30)



while True:
    dist1 = tof1.read()
    dist2 = tof2.read()
    print(f"{dist1} mm {dist2} mm")
