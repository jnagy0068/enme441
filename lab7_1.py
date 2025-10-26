import socket
import RPi.GPIO as GPIO
import time

led_pins = [23, 24, 25]  
f = 1000           
brightness = [0, 0, 0]    
pwms = []

GPIO.setmode(GPIO.BCM)
for pin in led_pins:
    GPIO.setup(pin, GPIO.OUT)
    pwm = GPIO.PWM(pin, f)
    pwm.start(0)
    pwms.append(pwm)

def change_brightness(index, value):
    value = int(value)
    if value < 0:
        value = 0
    if value > 100:
        value = 100
    brightness[index] = value
    pwms[index].ChangeDutyCycle(value)

def web_page():
    html = """
        <html>
        <form action="/cgi-bin/range.py" method="POST">
            Brightness Level:<br>
            <input type="range" name="brightness" min ="0" max="100" value ="0"/> <br>
            <br>
            Select LED:<br>
            <input type="radio" name="led1" value="led1"> LED 1 <br>
            <input type="radio" name="led2" value="led2"> LED 2 <br>
            <input type="radio" name="led3" value="led3"> LED 3 <br>
            <br>
            <input type="submit" value="Change Brightness">
        </form>
        </html>
        """
    print(html)
    return (bytes(html,'utf-8'))   # convert string to UTF-8 bytes object
     
# Serve the web page to a client on connection:
def serve_web_page():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # TCP-IP socket
    s.bind(('', 80))
    s.listen(1)  # up to 3 queued connections
    while True:
        time.sleep(0.1)
        print('Waiting for connection...')
        conn, (client_ip, client_port) = s.accept()     # blocking call
        request = conn.recv(1024)                 # read request (required even if none)
        print(f'Connection from {client_ip}')   
        conn.send(b'HTTP/1.1 200 OK\r\n')         # status line 
        conn.send(b'Content-type: text/html\r\n') # header (content type)
        conn.send(b'Connection: close\r\n\r\n')   # header (tell client to close)
        # send body in try block in case connection is interrupted:
        try:
            conn.sendall(web_page())                    # body
        finally:
            conn.close()

serve_web_page()
