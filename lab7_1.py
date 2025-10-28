import socket
import threading
import RPi.GPIO as GPIO
import time

led_pins = [17, 27, 22]  
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

def parsePOSTdata(data):
    data = data.decode('utf-8')
    data_dict = {}
    idx = data.find('\r\n\r\n')+4
    data = data[idx:]
    data_pairs = data.split('&')
    for pair in data_pairs:
        key_val = pair.split('=')
        if len(key_val) == 2:
            data_dict[key_val[0]] = key_val[1]
    return data_dict

def web_page(selected_led=0):
    c0 = 'checked' if selected_led == 0 else ''
    c1 = 'checked' if selected_led == 1 else ''
    c2 = 'checked' if selected_led == 2 else ''
    html = """
        <html>
        <form action="/cgi-bin/range.py" method="POST">
            Brightness Level:<br>
            <input type="range" name="brightness" min="0" max="100" value="{brightness[selected_led]}"> {brightness[selected_led]}%<br><br>
            Select LED:<br>
            <input type="radio" name="led" value="0" {c0}> LED 1 ({brightness[0]}%)<br>
            <input type="radio" name="led" value="1" {c1}> LED 2 ({brightness[1]}%)<br>
            <input type="radio" name="led" value="2" {c2}> LED 3 ({brightness[2]}%)<br>
            <br>
            <input type="submit" value="Change Brightness">
        </form>
        </html>
        """
    print(html)
    return (bytes(html,'utf-8'))
     
def serve_web_page():
    while True:
        print("Waiting for connection...")
        conn, (client_ip, client_port) = s.accept()
        print(f"Connection from {client_ip}:{client_port}")
        client_message = conn.recv(2048).decode('utf-8')
        print(f"Message from client:\n{client_message}")

        data_dict = parsePOSTdata(client_message)
        if 'led' in data_dict and 'brightness' in data_dict:
            led = int(data_dict['led'])
            value = int(data_dict['brightness'])
            change_brightness(led, value)

            conn.send(b'HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n')
            conn.sendall(web_page(led, value))
        else:
            conn.send(b'HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n')
            conn.sendall(web_page())

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('', 80))
s.listen(3)

webpageThread = threading.Thread(target=serve_web_page, daemon=True)
webpageThread.start()

try:
    while True:
        sleep(1)
except KeyboardInterrupt:
    print('\nExiting')
    for pwm in pwms: 
        pwm.stop()
    GPIO.cleanup()
    s.close()
