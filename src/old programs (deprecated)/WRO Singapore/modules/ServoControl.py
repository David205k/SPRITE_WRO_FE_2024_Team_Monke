import modules.PwmControl as PwmControl

class servo:

    def __init__(self, servoPin, startAng, minDuty, maxDuty):
        self.pin = servoPin
        self.startAng = startAng
        self.minDuty = minDuty
        self.maxDuty = maxDuty

        self.servoPwm = PwmControl.pwm(pin=servoPin, freq=50, startDuty= startAng * (maxDuty/180))

    def write(self, ang):

        if ang > 180:
            ang = 180

        if ang < 0:
            ang = 0

        duty = (ang / 180.0) * (self.maxDuty-self.minDuty) + self.minDuty
        self.servoPwm.setPwm(duty)    

