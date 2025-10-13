import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

class Shifter:
    def __init__(self, serialPin, clockPin, latchPin):
      self.serialPin = serialPin
      self.clockPin = clockPin
      self.latchPin = latchPin
      GPIO.setup(self.serialPin, GPIO.OUT)
      GPIO.setup(self.clockPin, GPIO.OUT, initial=0)
      GPIO.setup(self.latchPin, GPIO.OUT, initial=0)
    def _ping(self, pin):
      GPIO.output(pin, 1)
      time.sleep(0)
      GPIO.output(pin, 0)
  def shiftByte(self, pattern):
      for i in range(8):
            GPIO.output(self.serialPin, pattern & (1 << i))
            self._ping(self.clockPin)
        self._ping(self.latchPin)

try:
    while 1: 
        pass
except KeyboardInterrupt:
    GPIO.cleanup()
