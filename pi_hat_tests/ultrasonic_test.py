import RPi.GPIO as GPIO
import time
GPIO.setwarnings(False)     # Ignore warning for now
GPIO.setmode(GPIO.BOARD)    # Use physical pin numberin

TRIG = 12   # define input and output pins (NOT GPIO number)
ECHO = 11

print("setting up...")
# set up the input and output pins
GPIO.setup(TRIG, GPIO.OUT)
GPIO.output(TRIG, False)
GPIO.setup(ECHO, GPIO.IN)

print("sleeping...")

time.sleep(.5)              # let the sensor initialize
time_start = time.time()


def read():
    # trigger a reading
    print("reading...")
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)


    # find the start and end of the ultrasonic pulse
    print("finding start and end...")
    while GPIO.input(ECHO) == 0:
       start_time = time.time()
    while GPIO.input(ECHO) == 1:
       end_time   = time.time()
    #time_end = time.time()


    # Speed of sound 34300 cm/sec
    print("calculating...")
    total_distance = (end_time- start_time) * 34300
    time_taken = end_time-time_start
    # Divide by 2, account for return trip for signal
    print (f'Distance Away: {total_distance/2:.2f} cm')
    print (f'Taken in {end_time - start_time:.3f} seconds')
    return total_distance/2, time_taken

while True:
    read()
    
GPIO.cleanup()              # reset the GPIO 