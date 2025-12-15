import time
import threading
import socket
import json
from shifter import Shifter
import RPi.GPIO as GPIO
import os
from urllib.parse import parse_qs
import math
import multiprocessing

# ---------------- Config / Units (centimeters)
turret_height_self = 9.0
turret_height_other = 0.0

# --- GPIO Setup ---
GPIO.setmode(GPIO.BCM)
laser = 22
GPIO.setup(laser, GPIO.OUT)
GPIO.output(laser, GPIO.LOW)

# --- Shared Memory for Shifter bits ---
myArray = multiprocessing.Array('i', 2)

# --- Load positions from local JSON ---
positions = {}

def load_positions():
    global positions
    script_dir = os.path.dirname(os.path.realpath(__file__))
    filename = os.path.join(script_dir, "test_positions.json")
    if os.path.exists(filename):
        try:
            with open(filename, "r") as f:
                positions = json.load(f)
            print("Loaded JSON position file successfully:")
            print(json.dumps(positions, indent=2))
        except Exception as e:
            print("Error loading JSON:", e)
    else:
        print("JSON file not found at:", filename)

load_positions()

# --- Turret and Calibration Data ---
calibration = {"az_offset": 0.0, "el_offset": 0.0}
self_team = {"id": None}
zero_positions = {"m1": 0.0, "m2": 0.0}

def normalize_deg(angle):
    """Normalize angle to [-180, 180)."""
    return ((angle + 180.0) % 360.0) - 180.0

# --- Stepper Motor Class ---
class Stepper:
    seq = [0b0001,0b0011,0b0010,0b0110,0b0100,0b1100,0b1000,0b1001]
    delay = 500
    steps_per_degree = 4*1024/360

    def __init__(self, shifter, lock, index):
        self.s = shifter
        self.lock = lock
        self.index = index
        self.angle = 0.0
        self.step_state = 0
        self.shifter_bit_start = 4*index
        self._thread = None

    def _sgn(self, x):
        return 0 if x == 0 else int(abs(x)/x)

    def _step(self, direction):
        # --- Flip azimuth direction physically ---
        if self.index == 1:   # azimuth motor
            direction *= -1

        with self.lock:
            self.step_state = (self.step_state + direction) % 8
            myArray[self.index] &= ~(0b1111 << self.shifter_bit_start)
            myArray[self.index] |= (Stepper.seq[self.step_state] << self.shifter_bit_start)
            final = 0
            for val in myArray:
                final |= val
            self.s.shiftByte(final)
        self.angle = (self.angle + direction/Stepper.steps_per_degree) % 360.0
        time.sleep(Stepper.delay / 1e6)

    def _rotate(self, delta):
        direction = self._sgn(delta)
        steps = int(abs(delta) * Stepper.steps_per_degree)
        for _ in range(steps):
            self._step(direction)

    def rotate(self, delta):
        t = threading.Thread(target=self._rotate, args=(delta,), daemon=True)
        t.start()
        self._thread = t
        return t

    def goAngle(self, target):
        delta = (target - self.angle + 540.0) % 360.0 - 180.0
        return self.rotate(delta)

    def zero(self):
        self.angle = 0.0

# --- Laser ---
def test_laser():
    GPIO.output(laser, GPIO.HIGH)
    time.sleep(1)
    GPIO.output(laser, GPIO.LOW)

# --- Calibration & Zero Functions ---
def save_zero(m1, m2):
    zero_positions["m1"] = m1.angle
    zero_positions["m2"] = m2.angle
    print("Saved zero_positions:", zero_positions)

def return_to_zero(m1, m2):
    tgt_m1 = zero_positions.get("m1", 0.0)
    tgt_m2 = zero_positions.get("m2", 0.0)
    p1 = m1.goAngle(tgt_m1)
    p2 = m2.goAngle(tgt_m2)
    p1.join()
    p2.join()
    print("Returned to zero:", zero_positions)

# --- Aim at Team ---
def aim_at_team(m1, m2, target_team):
    if self_team["id"] is None:
        print("Self team not set")
        return
    if target_team not in positions.get("turrets", {}):
        print("Target team not in JSON:", target_team)
        return

    st = self_team["id"]
    th_self = positions["turrets"][st]["theta"]
    r_self  = positions["turrets"][st]["r"]

    th_tgt  = positions["turrets"][target_team]["theta"]
    r_tgt   = positions["turrets"][target_team]["r"]

    x_self = r_self * math.cos(th_self)
    y_self = r_self * math.sin(th_self)
    z_self = turret_height_self

    x_tgt  = r_tgt * math.cos(th_tgt)
    y_tgt  = r_tgt * math.sin(th_tgt)
    z_tgt  = turret_height_other

    dx = x_tgt - x_self
    dy = y_tgt - y_self
    dz = z_tgt - z_self

    # --- Compute azimuth and elevation relative to zero ---
    az_deg = math.degrees(math.atan2(dy, dx)) + calibration["az_offset"]
    el_deg = -math.degrees(math.atan2(dz, math.hypot(dx, dy))) + calibration["el_offset"]

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
    def jog_buttons(name):
        buttons = [-90,-45,-15,-5,-1,1,5,15,45,90]
        html_buttons = ""
        for b in buttons:
            html_buttons += f'<button name="{name}_jog" value="{b}">{b:+}°</button> '
        return html_buttons

    html = f"""
    <html>
    <body style="font-family:Arial;text-align:center;">
        <h2>Laser Turret Control</h2>
        <div>Elevation (m1): {m1_angle:.2f}°</div>
        <div>Azimuth (m2): {m2_angle:.2f}°</div>
        <form method="POST">
            <h3>Set Team</h3>
            <input name="self_team"><button name="set_self_team">Set</button><br>

            <h3>Aim at Team</h3>
            <input name="team_box"><button name="aim_team">Aim</button><br>

            <h3>Manual</h3>
            Az <input name="m2" value="{m2_angle:.2f}">{jog_buttons('m2')}<br>
            El <input name="m1" value="{m1_angle:.2f}">{jog_buttons('m1')}<br>
            <button>Rotate Motors</button><br>

            <h3>Calibration</h3>
            <button name="save_zero">Save Zero</button>
            <button name="return_zero">Return Zero</button><br>

            <h3>Laser</h3>
            <button name="laser">Test Laser</button>
        </form>
    </body>
    </html>
    """
    return html.encode()

# --- Web Server ---
def serve_web(m1, m2):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('', 8080))
    s.listen(3)
    print("Web server running on port 8080")

    while True:
        conn, _ = s.accept()
        try:
            msg = conn.recv(8192).decode(errors='ignore')
        except:
            conn.close()
            continue

        if msg.startswith("POST"):
            data = parsePOSTdata(msg)
            if "set_self_team" in data and data.get("self_team","").strip():
                self_team["id"] = data["self_team"].strip()
                print("This turret's team set to:", self_team["id"])

            if "aim_team" in data and data.get("team_box","").strip():
                aim_at_team(m1, m2, data["team_box"].strip())

            for k in ("m1_jog","m2_jog"):
                if k in data:
                    try:
                        delta = float(data[k])
                        p = (m1 if "m1" in k else m2).rotate(delta)
                        p.join()
                    except: pass

            if any(v=="Rotate Motors" for v in data.values()):
                if "m1" in data and data["m1"].strip():
                    try: m1.goAngle(float(data["m1"])).join()
                    except: pass
                if "m2" in data and data["m2"].strip():
                    try: m2.goAngle(float(data["m2"])).join()
                    except: pass

            if "save_zero" in data: save_zero(m1, m2)
            if "return_zero" in data: return_to_zero(m1, m2)
            if "laser" in data: test_laser()

        try:
            response = web_page(m1.angle, m2.angle)
            conn.send(b"HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n")
            conn.sendall(response)
        except:
            pass
        conn.close()

# --- Main ---
if __name__ == "__main__":
    s = Shifter(23,24,25)
    lock1 = threading.Lock()
    lock2 = threading.Lock()
    m1 = Stepper(s, lock1, 0)  # elevation
    m2 = Stepper(s, lock2, 1)  # azimuth

    zero_positions["m1"] = m1.angle
    zero_positions["m2"] = m2.angle
    print("Initial zero_positions:", zero_positions)

    t = threading.Thread(target=serve_web, args=(m1, m2), daemon=True)
    t.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        GPIO.cleanup()
        print("Exiting")
