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

    def toggle_wrap(self):
        self.isWrapOn = not self.isWrapOn
        print(f"Wrap mode: {self.isWrapOn}")

    def toggle_speed(self):
        self._speedBoost = not self._speedBoost
        if self._speedBoost:
            self.timestep /= 3
        else:
            self.timestep *= 3
        print(f"Speed boost: {self._speedBoost}, timestep: {self.timestep:.3f}")
    
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

# Input buttons
s1 = 17  # Start/Stop
s2 = 27  # Toggle wrap
s3 = 22  # Speed boost

GPIO.setmode(GPIO.BCM)
GPIO.setup(s1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(s2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(s3, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

bug = Bug()

def button_start_stop(channel):
    if bug._running:
        bug.stop()
        print("Bug stopped.")
    else:
        bug.start()
        print("Bug started.")
def button_toggle_wrap(channel):
    bug.toggle_wrap()
def button_toggle_speed(channel):
    bug.toggle_speed()

GPIO.add_event_detect(s1, GPIO.RISING, callback=button_start_stop, bouncetime=300)
GPIO.add_event_detect(s2, GPIO.RISING, callback=button_toggle_wrap, bouncetime=300)
GPIO.add_event_detect(s3, GPIO.RISING, callback=button_toggle_speed, bouncetime=300)

try:
    while True:
        bug.step()

except KeyboardInterrupt:
    bug.stop()
    GPIO.cleanup()
