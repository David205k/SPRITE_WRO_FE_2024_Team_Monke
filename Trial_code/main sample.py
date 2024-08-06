# Import necessary functions for movement and magnetometer control
from movement import execute_lap, detect_pillar_color, reverse_direction
from magnetometer_control import Magnetometer

# Main function to control the robot
def main(pillars):
    magnetometer = Magnetometer()  # Initialize the magnetometer
    start_heading = magnetometer.get_heading()  # Get the initial heading
    last_pillar_color = None  # Initialize the last pillar color

    # Loop through 3 laps
    for lap in range(1, 4):
        # Reverse direction if the last pillar in lap 2 was red
        if lap == 3 and last_pillar_color == "red":
            print("Reversing direction for lap 3")
            reverse_direction()

        # Execute the current lap
        execute_lap(lap, last_pillar_color, pillars)

        # Detect the last pillar's color at the end of lap 2
        if lap == 2:
            last_pillar_color = detect_pillar_color()

        # Check if the lap is complete based on the magnetometer heading
        while not magnetometer.is_lap_complete(start_heading):
            pass  # Continue driving until the lap is complete

# Entry point of the program
if __name__ == "__main__":
    pillars = 3  # Set the number of pillars here
    main(pillars)
