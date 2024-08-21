import modules.PwmControl as PwmControl

class servo:

    def __init__(self, servoPin, startAng):
        self.pin = servoPin
        self.startAng = startAng

        self.servoPwm = PwmControl.pwm(pin=servoPin, freq=50, startDuty= startAng * (10/180))

    def write(self, ang):

        if ang > 180:
            ang = 180
        if ang < 0:
            ang = 0

        duty = (ang/180) * (10-2) + 2
        self.servoPwm.setPwm(duty)



