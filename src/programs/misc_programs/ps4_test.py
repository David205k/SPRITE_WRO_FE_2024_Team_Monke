"""
This program demonstrates how to read input values from a bluetooth controller using pygame. 
"""
import pygame

# Initialize pygame and the joystick
pygame.init()
pygame.joystick.init()

# Assuming there's one controller connected
joystick = pygame.joystick.Joystick(0)
joystick.init()

print("Controller name:", joystick.get_name())

try:
    while True:
        # Get events from the controller
        pygame.event.pump()

        # Access joystick and button values
        axis_0 = joystick.get_axis(0)  # Left stick horizontal
        axis_1 = joystick.get_axis(1)  # Left stick vertical
        axis_2 = joystick.get_axis(2)  # Right stick horizontal
        axis_3 = joystick.get_axis(3)  # Right stick vertical

        button_x = joystick.get_button(0)  # X button
        button_circle = joystick.get_button(1)  # Circle button

        # Print the values for testing
        print(f"Axis 0: {axis_0}, Axis 1: {axis_1}, X button: {button_x}")

except KeyboardInterrupt:
    print("Exiting...")

finally:
    joystick.quit()
    pygame.quit()