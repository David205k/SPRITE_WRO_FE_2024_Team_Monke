import RPi.GPIO as GPIO
import time
import modules.Tb6612fngControl as Tb6612fng

# Pin no
sensorPin = 24
# Variables
lastPulseTime = 0 # time of lsat pulse
periodBetweenPulses = 0 # time period between two pulse
lastTime = 0 #the last time loop was executed
wheelCircumference = 0.080  # Wheel circumference (m)
debounceDelay = 50 / 1000.0  # Delay 50 ms (converted to seconds)
noPulseTimeout = 2.0  # Timeout 2000 ms (converted to seconds)
lastInterrupted = 0 # last time an interupt happened
pulseCount = 0 # Pulse counter to calculate RPM
periods = [0] * 5####
readIndex = 0
total = 0 # total of period
averagePeriod = 0 # avg period

car = Tb6612fng.motor(stby=37, pwmA=35, ai1=36, ai2=40) 

def setup():
    GPIO.setwarnings(False) # turn off warnings for pins (if pins were previously used and not released properly there will be warnings)
    GPIO.setmode(GPIO.BOARD) # pin name convention used is pin numbers on board

    GPIO.setup(sensorPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(sensorPin, GPIO.FALLING, callback=PulseCounter)


def PulseCounter(channel):
    global lastPulseTime, periodBetweenPulses, lastInterrupted, pulseCount
    global periods, Index, total, averagePeriod
    
    currentTime = time.time() * 1000000  # Current time in microseconds
    periodBetweenPulses = currentTime - lastPulseTime
    lastPulseTime = currentTime
    
    debounceTime = time.time()
    if debounceTime - lastInterrupted > debounceDelay:	
        pulseCount += 1
        update_periods(periodBetweenPulses)
        lastInterrupted = debounceTime

def update_periods(period):
    global periods, readIndex, total, averagePeriod
    
    total -= periods[Index] # subtract out old period from the total
    periods[Index] = period # store the new period in the array
    total += periods[Index] # add the new period to the total
    Index = (Index + 1) % len(periods) #next increment
    averagePeriod = total / len(periods) # avg period


def RPM_Calc(): #formula for rpm
    if averagePeriod > 0:
        return 60000000.0 / averagePeriod  # Convert period to RPM
    return 0

def Dist_Calc(rpm): #formula for distace
    return (rpm / 60.0) * wheelCircumference / 1000.0  # Convert from mm to m

def Terminal_Display(): # values of pulses rpm and distance outputed on the terminal
    rpm = RPM_Calc()
    distance = Dist_Calc(rpm)
    print(f"Pulses: {pulseCount}, RPM: {rpm}, Distance: {distance} meters")


def reset_pulseCount():  # Reset pulse count and update lastTime
    global pulseCount
    pulseCount = 0


def main():
    global lastTime, averagePeriod
    
    car.speed(20)

    while True:
        currentTime = time.time()
        
        # Update output every 100 milliseconds
        if currentTime - lastTime >= 0.1:
            rpm = RPM_Calc() * 1/30
            distance = Dist_Calc(rpm) / 1000 # distance in mm
            print(f"Pulses: {pulseCount}, RPM: {rpm}, Distance: {distance} meters")

            reset_pulseCount()
            lastTime = currentTime 

        # Handle case where no pulses are detected
        if currentTime - lastInterrupted > noPulseTimeout:
            averagePeriod = 0



def cleanup_gpio():
    GPIO.cleanup()

if __name__ == "__main__":
    try:
        setup()
        main()
    except KeyboardInterrupt:
        cleanup_gpio()



