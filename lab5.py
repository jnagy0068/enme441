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
    time = time.time()
    for y in pins:
      i = 0
      pwm[y].ChangeDutyCycle(((math.sin(2 * math.pi * f * time - i*math.pi/9))**2)*100) 
      i += 1
    pass
except KeyboardInterrupt:  
  print('\nExiting')
for z in pins:
  pwm[z].stop()
GPIO.cleanup()
