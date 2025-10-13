import time
import random
import RPi.GPIO as GPIO
from shifter import Shifter

dataPin, latchPin, clockPin = 23, 24, 25
led_shifter = Shifter(dataPin, clockPin, latchPin)

position = 3 
dt = 0.05

try:
    while True:
        pattern = 1 << position
        led_shifter.shiftByte(pattern)

        step = random.choice([-1, 1])
        position += step

        if position < 0:
            position = 0
        elif position > 7:
            position = 7

        time.sleep(dt)

except KeyboardInterrupt:
    GPIO.cleanup()
