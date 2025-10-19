import time
import random
import RPi.GPIO as GPIO
from shifter import Shifter

class Bug:
    def __init__(self, timestep=0.1, x=3, isWrapOn=False):
        self.timestep = timestep
        self.x = x
        self.isWrapOn = isWrapOn
        self.__shifter = Shifter(23, 24, 25)
        self._running = False

    def start(self):
        self._running = True

    def stop(self):
        self._running = False
        self.__shifter.shiftByte(0b00000000)
    
    def step(self):
        if not self._running:
            return
        pattern = 1 << self.x
        self.__shifter.shiftByte(pattern)
        step = random.choice([-1, 1])
        self.x += step
        if self.isWrapOn:
            self.x %= 8
        else:
            if self.x < 0:
                self.x = 0
            elif self.x > 7:
                self.x = 7

        time.sleep(self.timestep)

dataPin = 23
latchPin = 24
clockPin = 25

s1 = 17  # Start/Stop
s2 = 27  # Toggle wrap
s3 = 22  # Speed boost

for pin in [s1, s2, s3]:
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

bug = Bug()
last_s2_state = GPIO.input(s2)

def toggle_wrap(channel):
    bug.isWrapOn = not bug.isWrapOn
    print(f"Wrap mode toggled: {bug.isWrapOn}")

GPIO.add_event_detect(s2, GPIO.RISING, callback=toggle_wrap, bouncetime=300)

try:
    while True:
        if GPIO.input(s1):
            if not bug._running:
                bug.start()
        else:
            if bug._running:
                bug.stop()
        if GPIO.input(s3):
            current_step = bug.timestep / 3
        else:
            current_step = bug.timestep
        bug.step(current_step)
except KeyboardInterrupt:
    bug.stop()
    GPIO.cleanup()
