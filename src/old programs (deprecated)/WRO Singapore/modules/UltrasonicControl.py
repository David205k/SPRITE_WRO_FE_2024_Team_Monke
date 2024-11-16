import RPi.GPIO as GPIO
import time
import threading

# GPIO setup
GPIO.setmode(GPIO.BOARD)

class Sensor:
    """
    Class for using an Ultrasonic Sensor. 

    Each ultrasonic sensor object is run in a parallel thread. The more sensors added, 
    the more lag will be created.
    """
    def __init__(self, trig: int, echo: int):
        """
        Parameters
        ----------
        trig: int 
            Trigger pin of US sensor
        echo: int 
            Echo pin of US sensor
        """

        self.trig = trig
        self.echo = echo
        self.distance = 0

        self._stop_event = threading.Event()  # For stopping the thread gracefully
        self._thread = threading.Thread(target=self.measure_distance, daemon=True)

        # Setup GPIO pins for the sensor
        GPIO.setup(self.trig, GPIO.OUT)
        GPIO.setup(self.echo, GPIO.IN)

    def start(self):
        """Start the sensor measurement in a separate thread."""
        self._thread.start()

    def stop(self):
        """Stop the sensor measurement."""
        self._stop_event.set()
        self._thread.join()

    def measure_distance(self):
        """Continuously measure distance in a loop."""
        while not self._stop_event.is_set():

            # Trigger pulse
            GPIO.output(self.trig, True)
            time.sleep(0.00001)
            GPIO.output(self.trig, False)
            
            # Wait for echo
            start_time, end_time = time.time(), time.time()
            
            t = time.time()
            # Record the start time
            while GPIO.input(self.echo) == 0:
                start_time = time.time()

                if time.time()-t >= 0.05: # break out of loop in case we missed the echo
                    break

            # Record the end time
            while GPIO.input(self.echo) == 1:
                end_time = time.time()

            # Calculate the distance
            time_elapsed = end_time - start_time
            self.distance = (time_elapsed * 34300) / 2  # Distance in cm

            time.sleep(0.1)

    def get_distance(self):
        """Return the most recent distance measured by the sensor."""
        return self.distance
