import RPi.GPIO as GPIO
import math
GPIO.setmode(GPIO.BCM)

pins = [5, 6, 12, 13, 16, 19, 20, 21, 25, 26]    # GPIO pin number
f = 500     # frequency (Hz)

for x in pins:
  GPIO.setup(x, GPIO.OUT)
  pwm = GPIO.PWM(x, f)        # create PWM object

try:
  while True:
    pwm.start((sin(2 * math.pi * f * time.time()))**2)            # initiate PWM object
    pass
except KeyboardInterrupt:   # stop gracefully on ctrl-C
  print('\nExiting')

pwm.stop()
GPIO.cleanup()
