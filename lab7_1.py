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
    value = max(0, min(100, int(value)))
    brightness[index] = value
    pwms[index].ChangeDutyCycle(value)

def parsePOSTdata(data):
    data_dict = {}
    idx = data.find('\r\n\r\n') + 4
    post_data = data[idx:]
    data_pairs = post_data.split('&')
    for pair in data_pairs:
        key_val = pair.split('=')
        if len(key_val) == 2:
            data_dict[key_val[0]] = key_val[1]
    return data_dict

def web_page(selected_led=0):
    c0 = 'checked' if selected_led == 0 else ''
    c1 = 'checked' if selected_led == 1 else ''
    c2 = 'checked' if selected_led == 2 else ''
    html = f"""
    <html>
    <head><title>LED Control</title></head>
    <body style="font-family: Arial; text-align: center; margin-top: 50px;">
        <h2>LED Brightness Control</h2>
        <form action="/" method="POST">
            <label>Brightness Level:</label><br>
            <input type="range" name="brightness" min="0" max="100" value="{brightness[selected_led]}" oninput="this.nextElementSibling.value=this.value">
            <output>{brightness[selected_led]}</output>%<br><br>

            <label>Select LED:</label><br>
            <input type="radio" name="led" value="0" {c0}> LED 1 ({brightness[0]}%)<br>
            <input type="radio" name="led" value="1" {c1}> LED 2 ({brightness[1]}%)<br>
            <input type="radio" name="led" value="2" {c2}> LED 3 ({brightness[2]}%)<br><br>

            <input type="submit" value="Change Brightness">
        </form>
    </body>
    </html>
    """
    return bytes(html, 'utf-8')

def serve_web_page():
    while True:
        print("Waiting for connection...")
        conn, (client_ip, client_port) = s.accept()
        print(f"Connection from {client_ip}:{client_port}")
        
        client_message = conn.recv(2048).decode('utf-8', errors='ignore')
        print(f"Message from client:\n{client_message}")

        if client_message.startswith('POST'):
            data_dict = parsePOSTdata(client_message)
        else:
            data_dict = {}

        if 'led' in data_dict and 'brightness' in data_dict:
            led = int(data_dict['led'])
            value = int(data_dict['brightness'])
            change_brightness(led, value)
            response = web_page(led)
        else:
            response = web_page()

        conn.send(b'HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n')
        conn.sendall(response)
        conn.close()

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('', 8080))
s.listen(3)

webpageThread = threading.Thread(target=serve_web_page, daemon=True)
webpageThread.start()

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print('\nExiting')
    for pwm in pwms:
        pwm.stop()
    GPIO.cleanup()
    s.close()
