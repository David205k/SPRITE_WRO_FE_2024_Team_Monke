# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

""" Display compass heading data five times per second """
from math import atan2, degrees
import board
import adafruit_lis3mdl

class compass:

    def __init__(self):

        i2c = board.I2C()  # uses board.SCL and board.SDA

        # i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
        self.lis3mdl = adafruit_lis3mdl.LIS3MDL(i2c)

        self.startPos = 0

    def get_angle(self, relative=True):
        while True:
            try:
                magnet_x, magnet_y, _ = self.lis3mdl.magnetic
            except OSError:
                print("Unable to connect to compass.")
                continue
            break

        angle = degrees(atan2(magnet_y, magnet_x))
        if angle < 0:
            angle += 360
        
        if relative: # get angle relative to start position
            if 360 >= angle >= self.startPos: 
                angle -= self.startPos
            elif 0 <= angle < self.startPos:
                angle += (360 - self.startPos)        

        return angle

    def set_home(self, signal=True):
        """
        Set the reference position to read the compass angle from.
        """
        if signal == True:
            self.startPos = self.get_angle(relative=False)

