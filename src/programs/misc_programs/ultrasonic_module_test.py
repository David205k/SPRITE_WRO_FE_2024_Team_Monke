from ultrasonic_module import UltrasonicSensorHandler
import time

# Define GPIO pin numbers for each sensor
sensors = [
    {'trigger': 13, 'echo': 6},
    {'trigger':8, 'echo': 7},
    # Add more sensors as needed
]

# Initialize the ultrasonic sensor handler
sensor_handler = UltrasonicSensorHandler(sensors)

# Start the sensor reading thread
sensor_handler.start()

try:
    while True:
        distances = sensor_handler.get_distances()
        print("Distances:", distances)
        # time.sleep(1)  # Main loop delay
except KeyboardInterrupt:
    print("Stopping...")
finally:
    sensor_handler.stop()  # Ensure GPIO cleanup