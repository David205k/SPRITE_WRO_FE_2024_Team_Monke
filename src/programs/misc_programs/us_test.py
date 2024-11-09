from programs.monke_hat.UltrasonicControl import Sensor
from RPi import GPIO

# Create sensor instances with unique trigger and echo pins
sensor1 = Sensor(trig=15, echo=13)
sensor2 = Sensor(trig=33, echo=31)
sensor3 = Sensor(trig=24, echo=26)

# Start both sensors
sensor1.start()
sensor2.start()
sensor3.start()

distances = [0 for i in range(3)]

try:
    # Main loop to print distance readings
    while True:
        distances[0] = round(sensor1.get_distance())
        distances[1] = round(sensor2.get_distance())
        distances[2] = round(sensor3.get_distance())
        try:
            print(distances)
        except Exception:
            pass

except KeyboardInterrupt:
    print("Stopping sensor readings...")

finally:
    # Stop sensors and clean up GPIO
    sensor1.stop()
    sensor2.stop()
    sensor3.stop()  
    GPIO.cleanup()
    print("GPIO cleanup done.")
