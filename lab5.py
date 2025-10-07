import RPi.GPIO as GPIO
import math
import time
GPIO.setmode(GPIO.BCM)

pins = [5, 6, 12, 13, 16, 19, 20, 21, 25, 26]  
button_pin = 4
pwm = {}
f = 500  
direction = 1 

for x in pins:
  GPIO.setup(x, GPIO.OUT)
  pwm[x] = GPIO.PWM(x, f)  
  pwm[x].start(0)

GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def toggle_direction(direction):
    direction *= -1

GPIO.add_event_detect(button_pin, GPIO.RISING, callback=toggle_direction, bouncetime=300)

try:
  while True:
    t = time.time()
    for i, y in enumerate(pins):
      brightness = (math.sin(2 * math.pi * 0.2 * t - direction * i * math.pi / 9))**2 * 100
      pwm[y].ChangeDutyCycle(brightness)
    pass
except KeyboardInterrupt:  
  print('\nExiting')
for z in pins:
  pwm[z].stop()
GPIO.cleanup()
