import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

class Shifter:
    def __init__(self, dataPin, latchPin, clockPin):
        self.dataPin = dataPin
        self.latchPin = latchPin
        self.clockPin = clockPin
        GPIO.setup(self.dataPin, GPIO.OUT)
        GPIO.setup(self.latchPin, GPIO.OUT, initial=0)
        GPIO.setup(self.clockPin, GPIO.OUT, initial=0)
    def _ping(self, pin):
        GPIO.output(pin, 1)
        time.sleep(0.00001)
        GPIO.output(pin, 0)
        time.sleep(0.00001)
    def shiftByte(self, pattern):
        for i in range(8):
            GPIO.output(self.dataPin, (pattern >> i) & 1)
            self._ping(self.clockPin)
        self._ping(self.latchPin)
