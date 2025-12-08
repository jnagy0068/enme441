import time
import multiprocessing
import socket
import threading
import json
from shifter import Shifter
import RPi.GPIO as GPIO
import requests
from urllib.parse import parse_qs

GPIO.setmode(GPIO.BCM)
laser = 22
GPIO.setup(laser, GPIO.OUT)
GPIO.output(laser, GPIO.LOW)

myArray = multiprocessing.Array('i', 2)

positions = {}
JSON_URL = "http://192.168.1.254:8000/positions.json"

def load_positions():
    global positions
    try:
        response = requests.get(JSON_URL, timeout=5)
        response.raise_for_status()
        positions = response.json()
        print("Loaded JSON position file successfully from IP:")
        print(json.dumps(positions, indent=2))
        print("Available turret keys:", list(positions.get("turrets", {}).keys()))
    except Exception as e:
        print("Error loading JSON from IP:", e)

load_positions()


class Stepper:
    seq = [0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001]
    delay = 500 
    steps_per_degree = 4 * 1024 / 360

    def __init__(self, shifter, lock, index):
        self.s = shifter
        self.lock = lock
        self.index = index
        self.angle = 0
        self.step_state = 0
        self.shifter_bit_start = 4 * index

    def _sgn(self, x):
        return 0 if x == 0 else int(abs(x) / x)

    def _step(self, direction):
        with self.lock:
            self.step_state = (self.step_state + direction) % 8

            myArray[self.index] &= ~(0b1111 << self.shifter_bit_start)
            myArray[self.index] |= (Stepper.seq[self.step_state] << self.shifter_bit_start)

            final = 0
            for val in myArray:
                final |= val

            self.s.shiftByte(final)

        self.angle = (self.angle + direction / Stepper.steps_per_degree) % 360
        time.sleep(Stepper.delay / 1e6)

    def _rotate(self, delta):
        direction = self._sgn(delta)
        steps = int(abs(delta) * Stepper.steps_per_degree)
        for _ in range(steps):
            self._step(direction)

    def rotate(self, delta):
        p = multiprocessing.Process(target=self._rotate, args=(delta,))
        p.start()
        return p

    def goAngle(self, target):
        delta = (target - self.angle + 540) % 360 - 180
        return self.rotate(delta)

    def zero(self):
        self.angle = 0


def test_():
    GPIO.output(laser, GPIO.HIGH)
    time.sleep(3)
    GPIO.output(laser, GPIO.LOW)


def parsePOSTdata(data):
    idx = data.find('\r\n\r\n')
    if idx == -1:
        return {}
    post = data[idx+4:]
    parsed = parse_qs(post, keep_blank_values=True)
    simple = {k: v[0] for k, v in parsed.items()}
    return simple


def web_page(m1_angle, m2_angle):
    html = f"""
    <html>
    <head><title>Stepper Control</title></head>
    <body style="font-family: Arial; text-align:center; margin-top:40px;">
        <h2>Stepper Motor Angle Control</h2>

        <form action="/" method="POST">
            <label>Type Team Number:</label><br>
            <input type="text" name="team" placeholder="Enter team number"><br><br>
            <input type="submit" value="Get Position"><br><br>

            <label>Motor 1 Angle (degrees):</label><br>
            <input type="text" name="m1" value="{m1_angle}"><br><br>

            <label>Motor 2 Angle (degrees):</label><br>
            <input type="text" name="m2" value="{m2_angle}"><br><br>

            <input type="submit" value="Rotate Motors"><br><br>

            <!-- FIXED BUTTON -->
            <input type="submit" name="laser" value="Test Laser (3s)">
        </form>
    </body>
    </html>
    """
    return bytes(html, 'utf-8')

def serve_web(m1, m2):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('', 8080))
    s.listen(3)
    print("running at IP:8080")

    while True:
        conn, addr = s.accept()
        try:
            msg = conn.recv(4096).decode(errors='ignore')
        except Exception as e:
            print("Failed to read request:", e)
            conn.close()
            continue

        if msg.startswith("POST"):
            data = parsePOSTdata(msg)

            # team input (text)
            if "team" in data and data["team"].strip() != "":
                t = data["team"].strip()
                if t in positions.get("turrets", {}):
                    try:
                        r = positions["turrets"][t]["r"]
                        theta = positions["turrets"][t]["theta"]
                        print(f"\nTeam {t} Selected:")
                        print(f"Radius = {r}")
                        print(f"Theta  = {theta}\n")
                    except Exception as e:
                        print(f"Error reading turret data for team {t}: {e}")
                else:
                    print(f"Team '{t}' not found in positions. Available keys: {list(positions.get('turrets', {}).keys())}")

            # motor 1 control
            if "m1" in data and data["m1"].strip() != "":
                try:
                    m1_target = float(data["m1"])
                    p = m1.goAngle(m1_target)
                    p.join()
                except Exception as e:
                    print("Error rotating motor 1:", e)

            # motor 2 control
            if "m2" in data and data["m2"].strip() != "":
                try:
                    m2_target = float(data["m2"])
                    p = m2.goAngle(m2_target)
                    p.join()
                except Exception as e:
                    print("Error rotating motor 2:", e)

            # laser test button
            if "laser" in data:
                test_()

        # respond with the page showing current motor angles
        try:
            response = web_page(m1.angle, m2.angle)
            conn.send(b'HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n\r\n')
            conn.sendall(response)
        except Exception as e:
            print("Failed to send response:", e)

        conn.close()


if __name__ == '__main__':
    # create shifter and steppers as before
    s = Shifter(23, 24, 25)
    lock1 = multiprocessing.Lock()
    lock2 = multiprocessing.Lock()

    m1 = Stepper(s, lock1, 0)
    m2 = Stepper(s, lock2, 1)

    m1.zero()
    m2.zero()

    # start web server thread
    t = threading.Thread(target=serve_web, args=(m1, m2), daemon=True)
    t.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting.")
