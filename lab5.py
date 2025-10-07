import RPi.GPIO as GPIO
import math
import time
GPIO.setmode(GPIO.BCM)

pins = [5, 6, 12, 13, 16, 19, 20, 21, 25, 26]  
pwm = {}
f = 500  

for x in pins:
  GPIO.setup(x, GPIO.OUT)
  pwm[x] = GPIO.PWM(x, f)  
  pwm[x].start(0)

try:
  while True:
    t = time.time()
    for i, y in enumerate(pins):
      phi = direction * i * phase_step
      brightness = (math.sin(2 * math.pi * f_wave * t - phi))**2 * 100
      pwm[y].ChangeDutyCycle(brightness)
    pass
except KeyboardInterrupt:  
  print('\nExiting')
for z in pins:
  pwm[z].stop()
GPIO.cleanup()
