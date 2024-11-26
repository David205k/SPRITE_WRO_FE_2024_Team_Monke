import pygame

class RC_control:

    def __init__(self):
        # Initialise joystick object and controller variables

        # Initialize pygame and the joystick
        pygame.init()
        pygame.joystick.init()

        # Assuming there's one controller connected
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        print("Controller name:", self.controller.get_name())

        # joystick values
        self.joysticks = {"left":[0,0],  # [horizontal-axis value, vertical axis value]
                          "right":[0,0]}

        # button values
        self.buttons = {"X":[], # [prev_val, cur_val, state]
                        "O":[],
                        "TRI":[],
                        "SQR":[],
        }
        
        for key in self.buttons:
            self.buttons[key] = [0,0,False]
            
    def map(self, var, min1, max1, min2, max2):
        # map a value from one range to another
        return ((var - min1) / (max1-min1)) * (max2 - min2) + min2

    def read_controller(self):
        """
        Read ps4 controller values

        Reads joyticks, dpad and buttons. 
        """

        pygame.event.pump()

        # Access joystick and button values
        self.joysticks["left"] = [self.controller.get_axis(0), self.controller.get_axis(1)]
        self.joysticks["right"] = [self.controller.get_axis(3), self.controller.get_axis(2)]

        for i, key in enumerate(self.buttons):
            self.buttons[key][0] = self.buttons[key][1] # update prev values
            self.buttons[key][1] = self.controller.get_button(i) # 0:x, 1:O, 2:tri, 3:sqr

            if (self.buttons[key][0] == 0 and self.buttons[key][1] == 1): # check when button state goes from low to high
                self.buttons[key][2] = not self.buttons[key][2] # invert button state

        self.dpad = self.controller.get_hat(0) # dpad (list, shape is [4])

    def write_to_car(self, car_obj, max_speed=50, max_angle=45):
        """
        Maps controller joystick values to servo angle and motor speed.
        """

        speed = -round(self.map(self.joysticks["left"][1], -1, 1 , -max_speed, max_speed))
        servoAng = -round(self.map(self.joysticks["right"][0], -1, 1, -max_angle, max_angle))

        car_obj.motor.speed(speed)
        car_obj.servo.write(servoAng)



