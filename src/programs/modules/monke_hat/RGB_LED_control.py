import Pwm_control as Pwm_control

class LED:
    """
    Class for controlling an RGB LED

    Uses the RPi.GPIO library to control the LED pins

    Methods
    -------
    rgb(red, green, blue)  
    off()
    """

    def __init__(self, red: int, green: int, blue: int):
        """        
        Parameters
        ----------
        red : int
            Red LED pin (Physical RPI pin that is connected to it)
        green : int
            Green LED pin (Physical RPI pin that is connected to it)
        blue : int
            Blue LED pin (Physical RPI pin that is connected to it)
        """

        self.red = red
        self.green = green
        self.blue = blue    

        # set up rgb pins pwm
        self.pwmRed = Pwm_control.pwm(self.red, 100, 0)
        self.pwmGreen = Pwm_control.pwm(self.green, 100, 0)
        self.pwmBlue = Pwm_control.pwm(self.blue, 100, 0)


    def rgb(self, red: int = 0, green: int = 0 , blue: int = 0): # set led colour
        """
        Turns on and sets the rgb value of the LED.

        Parameters
        ----------
        red: int
            Red value (0-255)
        green: int
            Green value (0-255)
        blue: int
            Blue value (0-255)            
        """

        # cap the rgb values
        min(max(red, 0), 255)
        min(max(green, 0), 255)
        min(max(blue, 0), 255)

        self.pwmRed.setPwm(round(abs((red/255)*100)))
        self.pwmGreen.setPwm(round(abs(green/255)*100))
        self.pwmBlue.setPwm(round(abs(blue/255)*100))
    
    def off(self):
        """
        Turn off the RGB LED
        """

        self.pwmRed.setPwm(0)
        self.pwmGreen.setPwm(0)
        self.pwmBlue.setPwm(0)
