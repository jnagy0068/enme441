import RPi.GPIO as GPIO
import math
import time
GPIO.setmode(GPIO.BCM)

pins = [5, 6, 12, 13, 16, 19, 20, 21, 25, 26]  
button_pin = 4
pwm = {}
f = 500  
direction = 1       # 1 for forward, -1 for reverse
phase_step = math.pi / 9
f_wave = 0.2

for x in pins:
  GPIO.setup(x, GPIO.OUT)
  pwm[x] = GPIO.PWM(x, f)  
  pwm[x].start(0)

GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# --- Callback for button press ---
def toggle_direction(channel):
    global direction
    direction *= -1
    print(f"Direction changed to {'forward' if direction == 1 else 'reverse'}")

GPIO.add_event_detect(button_pin, GPIO.RISING, callback=toggle_direction, bouncetime=300)

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
