import RPi.GPIO as GPIO
import math
import time
GPIO.setmode(GPIO.BCM)

pins = [5, 6, 12, 13, 16, 19, 20, 21, 25, 26]    # GPIO pin number
pwm = {}
f = 500     # frequency (Hz)

for x in pins:
  GPIO.setup(x, GPIO.OUT)
  pwm[x] = GPIO.PWM(x, f)        # create PWM object

try:
  for y in pins:
    i = 0
    pwm[y].start(((math.sin(2 * math.pi * f * time.time() - i*math.pi()/9))**2)*100) 
    i += 1
  while True:
    pass
except KeyboardInterrupt:   # stop gracefully on ctrl-C
  print('\nExiting')
for z in pins:
  pwm[z].stop()
GPIO.cleanup()
