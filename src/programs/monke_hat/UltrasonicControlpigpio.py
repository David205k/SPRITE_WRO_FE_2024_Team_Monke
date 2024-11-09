import pigpio
import time
import threading

class Sensor:
    """
    Class for using an Ultrasonic Sensor with pigpio for better timing accuracy.

    Each ultrasonic sensor object is run in a parallel thread. The more sensors added, 
    the more lag might be created if the Pi's processing power is limited.
    """
    def __init__(self, trig: int, echo: int):
        """
        Parameters
        ----------
        trig: int 
            Trigger pin of the ultrasonic sensor
        echo: int 
            Echo pin of the ultrasonic sensor
        """

        self.trig = trig
        self.echo = echo
        self.distance = 0.0

        # Connect to pigpio daemon
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpio daemon is not running.")

        # Set up GPIO pins for the sensor
        self.pi.set_mode(self.trig, pigpio.OUTPUT)
        self.pi.set_mode(self.echo, pigpio.INPUT)
        self.pi.write(self.trig, 0)  # Ensure trigger is low

        # Thread management
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self.measure_distance, daemon=True)

    def start(self):
        """Start the sensor measurement in a separate thread."""
        self._thread.start()

    def stop(self):
        """Stop the sensor measurement and clean up."""
        self._stop_event.set()
        self._thread.join()
        self.pi.stop()  # Clean up pigpio resources

    def measure_distance(self):
        """Continuously measure distance in a loop."""
        while not self._stop_event.is_set():
            # Trigger the sensor
            self.pi.gpio_trigger(self.trig, 10)  # Send 10µs pulse

            # Wait for echo
            start_tick = end_tick = None
            max_time = time.time() + 0.05  # Timeout after 50ms

            # Record the start time (when echo goes high)
            while self.pi.read(self.echo) == 0:
                start_tick = self.pi.get_current_tick()
                if time.time() > max_time:
                    break

            # Record the end time (when echo goes low)
            while self.pi.read(self.echo) == 1:
                end_tick = self.pi.get_current_tick()
                if time.time() > max_time:
                    break

            # Calculate the distance if we got valid ticks
            if start_tick is not None and end_tick is not None:
                time_elapsed = pigpio.tickDiff(start_tick, end_tick) / 1e6  # Convert µs to seconds
                self.distance = (time_elapsed * 34300) / 2  # Distance in cm

            time.sleep(0.06)  # Small delay between readings

    def get_distance(self):
        """Return the most recent distance measured by the sensor."""
        return self.distance