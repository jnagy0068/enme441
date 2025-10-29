import socket
import threading
import RPi.GPIO as GPIO
import time

# --- LED SETUP ---
led_pins = [17, 27, 22]
f = 1000  # PWM frequency
brightness = [0, 0, 0]
pwms = []

GPIO.setmode(GPIO.BCM)
for pin in led_pins:
    GPIO.setup(pin, GPIO.OUT)
    pwm = GPIO.PWM(pin, f)
    pwm.start(0)
    pwms.append(pwm)


# --- FUNCTIONS ---
def change_brightness(index, value):
    """Clamp brightness 0–100 and update PWM duty cycle."""
    value = max(0, min(100, int(value)))
    brightness[index] = value
    pwms[index].ChangeDutyCycle(value)


def parsePOSTdata(data):
    """Extract POST data (led index and brightness) from HTTP request."""
    if isinstance(data, bytes):
        data = data.decode("utf-8")
    data_dict = {}
    idx = data.find("\r\n\r\n") + 4
    if idx < 4:
        return data_dict
    data = data[idx:]
    for pair in data.split("&"):
        key_val = pair.split("=")
        if len(key_val) == 2:
            data_dict[key_val[0]] = key_val[1]
    return data_dict


def web_page():
    """Return HTML page for LED control sliders."""
    html = f"""
    <html>
    <head>
        <style>
            body {{
                font-family: Arial, sans-serif;
                text-align: center;
                margin-top: 40px;
                background-color: #fafafa;
            }}
            .box {{
                border: 3px solid #555;
                border-radius: 12px;
                width: 420px;
                margin: 0 auto;
                padding: 25px;
                background-color: #fff;
                box-shadow: 0px 4px 10px rgba(0,0,0,0.1);
            }}
            .slider-container {{
                display: flex;
                align-items: center;
                justify-content: space-between;
                margin: 20px 0;
            }}
            .led-label {{
                width: 60px;
                font-weight: bold;
                text-align: left;
            }}
            input[type=range] {{
                flex: 1;
                margin: 0 10px;
                accent-color: #2979ff;
            }}
            .value {{
                width: 40px;
                font-weight: bold;
                text-align: right;
            }}
        </style>
    </head>
    <body>
        <div class="box">
            <h2>Raspberry Pi LED Control</h2>
            <div class="slider-container">
                <span class="led-label">LED1</span>
                <input type="range" id="led0" min="0" max="100" value="{brightness[0]}" oninput="updateLED(0, this.value)">
                <span id="val0" class="value">{brightness[0]}</span>
            </div>
            <div class="slider-container">
                <span class="led-label">LED2</span>
                <input type="range" id="led1" min="0" max="100" value="{brightness[1]}" oninput="updateLED(1, this.value)">
                <span id="val1" class="value">{brightness[1]}</span>
            </div>
            <div class="slider-container">
                <span class="led-label">LED3</span>
                <input type="range" id="led2" min="0" max="100" value="{brightness[2]}" oninput="updateLED(2, this.value)">
                <span id="val2" class="value">{brightness[2]}</span>
            </div>
        </div>

        <script>
            function updateLED(led, value) {{
                document.getElementById('val' + led).innerText = value;
                fetch("/", {{
                    method: "POST",
                    headers: {{
                        "Content-Type": "application/x-www-form-urlencoded"
                    }},
                    body: "led=" + led + "&brightness=" + value
                }});
            }}
        </script>
    </body>
    </html>
    """
    return bytes(html, "utf-8")


# --- WEB SERVER ---
def serve_web_page():
    while True:
        print("Waiting for connection...")
        conn, (client_ip, client_port) = s.accept()
        print(f"Connection from {client_ip}:{client_port}")
        client_message = conn.recv(2048).decode("utf-8", errors="ignore")
        print(f"Message from client:\n{client_message}")

        if "POST" in client_message:
            data_dict = parsePOSTdata(client_message)
            if "led" in data_dict and "brightness" in data_dict:
                led = int(data_dict["led"])
                value = int(data_dict["brightness"])
                change_brightness(led, value)

        conn.send(b"HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n")
        conn.sendall(web_page())
        conn.close()


# --- MAIN PROGRAM ---
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(("", 8080))  # Port 8080 to avoid needing sudo
s.listen(3)
print("Server running. Connect via http://<Pi_IP>:8080")

webpageThread = threading.Thread(target=serve_web_page, daemon=True)
webpageThread.start()

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\nExiting")
    for pwm in pwms:
        pwm.stop()
    GPIO.cleanup()
    s.close()
