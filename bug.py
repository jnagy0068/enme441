import time
import random
import RPi.GPIO as GPIO
from shifter import Shifter

dataPin = 23
latchPin = 24
clockPin = 25

position = 3  # start roughly in the middle
timeStep = 0.05  # 50 ms between updates

leds = Shifter(dataPin, latchPin, clockPin)

try:
    while True:
        pattern = 1 << position
        leds.shiftByte(pattern)

        step = random.choice([-1, 1])
        position += step

        if position < 0:
            position = 0
        elif position > 7:
            position = 7
        time.sleep(timeStep)

except KeyboardInterrupt:
    GPIO.cleanup()
