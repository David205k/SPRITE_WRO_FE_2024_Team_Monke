from math import *

class PID:
    """
    Class for creating a PID controller

    Methods
    -------
    PID(error)
        Returns an adjustemnt value using PID based on the error
    """

    def __init__(self, kp: float, ki: float, kd:float):
        """
        Parameters
        kp: float
            Proportional Gain
        kd: float
            Derivative Gain
        ki: float
            Integral Gain
        """

        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.error = 0
        self.prev_error = 0
        self.integral = 0

    def PID_control(self, error: float) -> float:
        """
        Returns an adjustment based on the error value given.

        Parameters
        ----------
        control_var: float
            variable to control
        set_point: float
            value to keep the variable about
        adjustment: float
            adjustment to variable
        """

        self.prev_error = self.error
        self.integral += self.error
        self.error = error

        adjustment = self.error*self.kp + (self.error-self.prev_error)*self.kd + self.integral*self.ki

        return adjustment
