import time
import multiprocessing
import socket
import threading
import json
from shifter import Shifter
import RPi.GPIO as GPIO
import requests
from urllib.parse import parse_qs
import math

# --- GPIO Setup ---
GPIO.setmode(GPIO.BCM)
laser = 22
GPIO.setup(laser, GPIO.OUT)
GPIO.output(laser, GPIO.LOW)

# --- Shared Memory for Stepper Shifter ---
myArray = multiprocessing.Array('i', 2)

# --- Load positions from IP ---
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

# --- Turret and Calibration Data ---
calibration = {"az_offset": 0.0, "el_offset": 0.0}
self_team = {"id": None}

# --- Stepper Motor Class ---
class Stepper:
    seq = [0b0001,0b0011,0b0010,0b0110,0b0100,0b1100,0b1000,0b1001]
    delay = 500
    steps_per_degree = 4*1024/360

    def __init__(self, shifter, lock, index):
        self.s = shifter
        self.lock = lock
        self.index = index
        self.angle = 0
        self.step_state = 0
        self.shifter_bit_start = 4*index

    def _sgn(self, x):
        return 0 if x == 0 else int(abs(x)/x)

    def _step(self, direction):
        with self.lock:
            self.step_state = (self.step_state + direction) % 8
            myArray[self.index] &= ~(0b1111 << self.shifter_bit_start)
            myArray[self.index] |= (Stepper.seq[self.step_state] << self.shifter_bit_start)
            final = 0
            for val in myArray:
                final |= val
            self.s.shiftByte(final)
        self.angle = (self.angle + direction/Stepper.steps_per_degree) % 360
        time.sleep(Stepper.delay/1e6)

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

# --- Laser Function ---
def test_laser():
    GPIO.output(laser, GPIO.HIGH)
    time.sleep(3)
    GPIO.output(laser, GPIO.LOW)

# --- Calibration Functions ---
def save_zero(m1, m2):
    calibration["el_offset"] = -m1.angle
    calibration["az_offset"] = -m2.angle
    print("Saved zero position:", calibration)

def return_to_zero(m1, m2):
    target_el = calibration["el_offset"]
    target_az = calibration["az_offset"]
    print("Returning to zero...")
    p1 = m1.goAngle(target_el)
    p2 = m2.goAngle(target_az)
    p1.join()
    p2.join()

# --- Correct Aim at Team for Circumference ---
def aim_at_team(m1, m2, target_team):
    if self_team["id"] is None:
        print("ERROR: Self team number not set.")
        return
    if target_team not in positions.get("turrets", {}):
        print("Team not found in positions:", target_team)
        return

    st = self_team["id"]
    if st not in positions["turrets"]:
        print("ERROR: This turret's team number not in positions:", st)
        return

    # Get turret positions in radians
    th_self = positions["turrets"][st]["theta"]
    th_tgt = positions["turrets"][target_team]["theta"]

    # Cartesian positions on the circle (radius arbitrary since it cancels)
    R = 1.0
    x_self = R * math.cos(th_self)
    y_self = R * math.sin(th_self)
    x_tgt  = R * math.cos(th_tgt)
    y_tgt  = R * math.sin(th_tgt)

    # Angle relative to turret's current pointing (toward center)
    az_rad = math.atan2(y_tgt - y_self, x_tgt - x_self) - (th_self + math.pi)
    az_deg = az_rad * 180.0 / math.pi

    # Apply calibration offsets
    az_deg += calibration["az_offset"]
    el_deg = calibration["el_offset"]

    print(f"Aiming at team {target_team}: az={az_deg:.2f}, el={el_deg:.2f}")

    p1 = m1.goAngle(el_deg)
    p2 = m2.goAngle(az_deg)
    p1.join()
    p2.join()

# --- POST Parsing ---
def parsePOSTdata(data):
    idx = data.find("\r\n\r\n")
    if idx == -1:
        return {}
    post = data[idx+4:]
    parsed = parse_qs(post, keep_blank_values=True)
    return {k:v[0] for k,v in parsed.items()}

# --- Web Page ---
def web_page(m1_angle, m2_angle):
    html = f"""
    <html>
    <head><title>Laser Turret</title></head>
    <body style="font-family: Arial; text-align:center; margin-top:40px;">
        <h2>Laser Turret Control</h2>
        <form action="/" method="POST">
            <h3>This Turret's Team Number</h3>
            <input type="text" name="self_team" placeholder="Your team number"><br>
            <input type="submit" name="set_self_team" value="Set This Turret's Team"><br>

            <h3>Aim at Another Team</h3>
            <input type="text" name="team_box" placeholder="Target team #"><br>
            <input type="submit" name="aim_team" value="Aim at Team"><br>

            <h3>Manual Motor Control</h3>
            <label>Azimuth:</label><br>
            <input type="text" name="m2" value="{m2_angle}"><br>
            <label>Elevation:</label><br>
            <input type="text" name="m1" value="{m1_angle}"><br>
            <input type="submit" value="Rotate Motors"><br>

            <h3>Calibration</h3>
            <input type="submit" name="return_zero" value="Return to Zero"><br>
            <input type="submit" name="save_zero" value="Save Current Position as Zero"><br>

            <h3>Laser</h3>
            <input type="submit" name="laser" value="Test Laser (3s)"><br>
        </form>
    </body>
    </html>
    """
    return bytes(html, "utf-8")

# --- Web Server ---
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
        except:
            conn.close()
            continue

        if msg.startswith("POST"):
            data = parsePOSTdata(msg)

            # Set self team
            if "set_self_team" in data and data.get("self_team", "").strip() != "":
                self_team["id"] = data["self_team"].strip()
                print("This turret's team set to:", self_team["id"])

            # Aim at another team
            if "aim_team" in data and data.get("team_box", "").strip() != "":
                aim_at_team(m1, m2, data["team_box"].strip())

            # Motor control
            if "m1" in data and data["m1"].strip() != "":
                try:
                    el = float(data["m1"])
                    p = m1.goAngle(el)
                    p.join()
                except: pass

            if "m2" in data and data["m2"].strip() != "":
                try:
                    az = float(data["m2"])
                    p = m2.goAngle(az)
                    p.join()
                except: pass

            # Calibration buttons
            if "return_zero" in data: return_to_zero(m1, m2)
            if "save_zero" in data: save_zero(m1, m2)

            # Laser button
            if "laser" in data: test_laser()

        try:
            response = web_page(m1.angle, m2.angle)
            conn.send(b"HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n\r\n")
            conn.sendall(response)
        except: pass

        conn.close()

# --- Main ---
if __name__ == "__main__":
    s = Shifter(23, 24, 25)
    lock1 = multiprocessing.Lock()
    lock2 = multiprocessing.Lock()

    m1 = Stepper(s, lock1, 0)  # elevation
    m2 = Stepper(s, lock2, 1)  # azimuth

    m1.zero()
    m2.zero()

    t = threading.Thread(target=serve_web, args=(m1, m2), daemon=True)
    t.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting.")
