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

# Input switches
s1 = 17  # Start/Stop
s2 = 27  # Toggle wrap
s3 = 22  # Speed boost

GPIO.setmode(GPIO.BCM)
GPIO.setup(s1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(s2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(s3, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

bug = Bug()

last_s2_state = GPIO.input(s2)

try:
    while True:
        if GPIO.input(s1):
            if not bug._running:
                bug.start()
        else:
            if bug._running:
                bug.stop()
        current_s2_state = GPIO.input(s2)
        if current_s2_state != last_s2_state and current_s2_state == 1:
            bug.isWrapOn = not bug.isWrapOn
            print(f"Wrap mode toggled: {bug.isWrapOn}")
        last_s2_state = current_s2_state
        if GPIO.input(s3):
            bug.timestep = 0.1 / 3
        else:
            bug.timestep = 0.1
        bug.step()

except KeyboardInterrupt:
    bug.stop()
    GPIO.cleanup()
