import time
import socket
import threading
import json
import math
import os
from urllib.parse import parse_qs

from shifter import Shifter
import RPi.GPIO as GPIO

# --------------------------------------------------
# Z-positions (cm)
# --------------------------------------------------
turret_height_self = 9.0
turret_height_other = 0.0

# --------------------------------------------------
# GPIO Setup
# --------------------------------------------------
GPIO.setmode(GPIO.BCM)
laser = 22
GPIO.setup(laser, GPIO.OUT)
GPIO.output(laser, GPIO.LOW)

# --------------------------------------------------
# Load positions from local JSON
# --------------------------------------------------
positions = {}

def load_positions():
    global positions
    script_dir = os.path.dirname(os.path.realpath(__file__))
    filename = os.path.join(script_dir, "test_positions.json")

    if os.path.exists(filename):
        with open(filename, "r") as f:
            positions = json.load(f)
        print("Loaded JSON:")
        print(json.dumps(positions, indent=2))
    else:
        print("JSON file not found")

load_positions()

# --------------------------------------------------
# Turret State
# --------------------------------------------------
self_team = {"id": None}
current_target_team = {"id": None}

zero_angles = {"m1": 0.0, "m2": 0.0}
calibration_offsets = {"m1": 0.0, "m2": 0.0}

# --------------------------------------------------
# Stepper Motor Class
# --------------------------------------------------
class Stepper:
    seq = [0b0001,0b0011,0b0010,0b0110,
           0b0100,0b1100,0b1000,0b1001]

    delay = 500  # microseconds
    steps_per_degree = 4 * 1024 / 360

    def __init__(self, shifter, lock, index):
        self.s = shifter
        self.lock = lock
        self.index = index
        self.angle = 0.0  # signed for elevation, modulo 360 for azimuth
        self.step_state = 0
        self.bit_start = 4 * index

    def _sgn(self, x):
        return 0 if x == 0 else int(abs(x) / x)

    def _step(self, direction):
        with self.lock:
            self.step_state = (self.step_state + direction) % 8
            bits = Stepper.seq[self.step_state] << self.bit_start
            final = self.s.last_value & ~(0b1111 << self.bit_start)
            final |= bits
            self.s.shiftByte(final)
            self.s.last_value = final

        # Update angle
        delta_angle = direction / Stepper.steps_per_degree
        if self.index == 0:  # elevation
            self.angle += delta_angle  # signed, no wrapping
        else:  # azimuth
            self.angle = (self.angle + delta_angle) % 360

        time.sleep(Stepper.delay / 1e6)

    def _rotate(self, delta):
        direction = self._sgn(delta)
        steps = int(abs(delta) * Stepper.steps_per_degree)
        for _ in range(steps):
            self._step(direction)

    def rotate(self, delta):
        t = threading.Thread(target=self._rotate, args=(delta,))
        t.start()
        return t

    def goAngle(self, target):
        if self.index == 0:  # elevation
            delta = target - self.angle
        else:  # azimuth
            delta = (target - self.angle + 540) % 360 - 180
        return self.rotate(delta)

    def zero(self):
        self.angle = 0.0

# --------------------------------------------------
# Laser
# --------------------------------------------------
def test_laser():
    GPIO.output(laser, GPIO.HIGH)
    time.sleep(1)
    GPIO.output(laser, GPIO.LOW)

# --------------------------------------------------
# Calibration
# --------------------------------------------------
def save_zero(m1, m2):
    zero_angles["m1"] = m1.angle
    zero_angles["m2"] = m2.angle
    calibration_offsets["m1"] = -m1.angle
    calibration_offsets["m2"] = -m2.angle

def return_to_zero(m1, m2):
    m1.goAngle(zero_angles["m1"] + calibration_offsets["m1"]).join()
    m2.goAngle(zero_angles["m2"] + calibration_offsets["m2"]).join()

# --------------------------------------------------
# Aim-at-Team with correct height and shortest-path
# --------------------------------------------------
def aim_at_team(m1, m2, target_team):
    if self_team["id"] is None:
        return
    if target_team not in positions.get("turrets", {}):
        return

    st = self_team["id"]
    if st not in positions["turrets"]:
        return

    current_target_team["id"] = target_team

    r_self = positions["turrets"][st]["r"]
    th_self = positions["turrets"][st]["theta"]
    r_tgt  = positions["turrets"][target_team]["r"]
    th_tgt = positions["turrets"][target_team]["theta"]

    x_self = r_self * math.cos(th_self)
    y_self = r_self * math.sin(th_self)
    x_tgt  = r_tgt * math.cos(th_tgt)
    y_tgt  = r_tgt * math.sin(th_tgt)

    dx = x_tgt - x_self
    dy = y_tgt - y_self
    dz = turret_height_other - turret_height_self  # negative if target below turret

    # Absolute azimuth (0–360°)
    az_deg = math.degrees(math.atan2(dy, dx)) + calibration_offsets["m2"]
    az_deg %= 360

    # Signed elevation (negative = down)
    el_deg = math.degrees(math.atan2(dz, math.hypot(dx, dy))) + calibration_offsets["m1"]

    # Shortest-path deltas
    delta_az = (az_deg - m2.angle + 540) % 360 - 180
    delta_el = el_deg - m1.angle  # signed

    print(f"Aiming from m2={m2.angle:.2f}° to az={az_deg:.2f}° → delta={delta_az:.2f}°")
    print(f"Aiming from m1={m1.angle:.2f}° to el={el_deg:.2f}° → delta={delta_el:.2f}°")

    m1.goAngle(m1.angle + delta_el).join()
    m2.goAngle(m2.angle + delta_az).join()
# --------------------------------------------------
# POST Parsing
# --------------------------------------------------
def parsePOSTdata(data):
    idx = data.find("\r\n\r\n")
    if idx == -1:
        return {}
    parsed = parse_qs(data[idx+4:], keep_blank_values=True)
    return {k:v[0] for k,v in parsed.items()}

# --------------------------------------------------
# Web Page
# --------------------------------------------------
def web_page(m1_angle, m2_angle):
    def jog_buttons(name):
        vals = [-90,-45,-15,-5,-1,-0.5,0.5,1,5,15,45,90]
        return " ".join(
            f'<button type="submit" name="{name}" value="{v}">{v:+}°</button>'
            for v in vals
        )

    return f"""
    <html><body style="font-family:Arial;text-align:center;">
    <h2>Laser Turret</h2>
    <p>Self: {self_team['id']} | Target: {current_target_team['id']}</p>

    <form method="POST">
        <h3>Set Team</h3>
        <input name="self_team">
        <button type="submit" name="set_self_team" value="1">Set</button>

        <h3>Aim</h3>
        <input name="team_box">
        <button type="submit" name="aim_team" value="1">Aim</button>

        <h3>Manual</h3>
        Az <input name="m2" value="{m2_angle:.2f}">{jog_buttons('m2_jog')}<br>
        El <input name="m1" value="{m1_angle:.2f}">{jog_buttons('m1_jog')}<br><br>
        <button type="submit" name="manual_move" value="1">Rotate</button>

        <h3>Calibration</h3>
        <button type="submit" name="save_zero" value="1">Save Zero</button>
        <button type="submit" name="return_zero" value="1">Return Zero</button>

        <h3>Laser</h3>
        <button type="submit" name="laser" value="1">Test</button>
    </form>
    </body></html>
    """.encode()

# --------------------------------------------------
# Web Server
# --------------------------------------------------
def serve_web(m1, m2):
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('', 8080))
    s.listen(3)

    print("Web server started on port 8080")

    while True:
        conn, _ = s.accept()
        msg = conn.recv(4096).decode(errors="ignore")
        if msg.startswith("POST"):
            d = parsePOSTdata(msg)

            # Set self team
            if "set_self_team" in d:
                self_team["id"] = d.get("self_team")

            # Auto-aim at team
            if "aim_team" in d:
                aim_at_team(m1, m2, d.get("team_box"))

            # Manual rotation input
            if "manual_move" in d:
                if "m1" in d and d["m1"]:
                    try: m1.goAngle(float(d["m1"])).join()
                    except: pass
                if "m2" in d and d["m2"]:
                    try: m2.goAngle(float(d["m2"])).join()
                    except: pass

            # Jog buttons
            for k in ("m1_jog","m2_jog"):
                if k in d:
                    try:
                        (m1 if "m1" in k else m2).rotate(float(d[k])).join()
                    except: pass

            # Calibration
            if "save_zero" in d: save_zero(m1, m2)
            if "return_zero" in d: return_to_zero(m1, m2)

            # Laser
            if "laser" in d: test_laser()

        conn.send(b"HTTP/1.1 200 OK\r\nContent-Type:text/html\r\n\r\n")
        conn.sendall(web_page(m1.angle, m2.angle))
        conn.close()

# --------------------------------------------------
# Main
# --------------------------------------------------
if __name__ == "__main__":
    shifter = Shifter(23,24,25)
    shifter.last_value = 0

    lock = threading.Lock()
    m1 = Stepper(shifter, lock, 0)  # elevation
    m2 = Stepper(shifter, lock, 1)  # azimuth

    m1.zero()
    m2.zero()

    threading.Thread(target=serve_web, args=(m1,m2),
                     daemon=True).start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        GPIO.cleanup()
