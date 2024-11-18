# repo: https://github.com/CoreElectronics/CE-PiicoDev-VL53L1X-MicroPython-Module
# module bought from: https://github.com/CoreElectronics/CE-PiicoDev-VL53L1X-MicroPython-Module
# default i2c addr: 0x29
# use lsmod to check i2c
# of i2cdetect -y <i2c bus number>


from PiicoDev_VL53L1X import PiicoDev_VL53L1X 

from RPi import GPIO

GPIO.setmode(GPIO.BCM)

left = 11
right = 8

GPIO.setup(left, GPIO.OUT)
GPIO.setup(right, GPIO.OUT)

GPIO.output(left, GPIO.LOW)
GPIO.output(right, GPIO.LOW)

GPIO.output(left, GPIO.HIGH)
tof1 = PiicoDev_VL53L1X(bus=1, sda=27, scl=28, freq=400_000 )
tof1.change_addr(0x30)

GPIO.output(right, GPIO.HIGH)
tof2 = PiicoDev_VL53L1X(bus=1, sda=27, scl=28, freq=400_000)
tof2.change_addr(0x31)

while True:
    dist1 = tof1.read()
    dist2 = tof2.read()
    print(f"{dist1} mm {dist2} mm")
