import time
import multiprocessing
import socket
import threading
import json
from shifter import Shifter
import RPi.GPIO as GPIO
import os
from urllib.parse import parse_qs
import math

# --- Z-positions (cm) ---
turret_height_self = 3.0
turret_height_other = 0.0

# --- GPIO Setup ---
GPIO.setmode(GPIO.BCM)
laser = 22
GPIO.setup(laser, GPIO.OUT)
GPIO.output(laser, GPIO.LOW)

# --- Shared Memory for Stepper Shifter ---
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
            print("Available turret keys:", list(positions.get("turrets", {}).keys()))
        except Exception as e:
            print("Error loading JSON:", e)
    else:
        print("JSON file not found at:", filename)
load_positions()

# --- Turret and Calibration Data ---
self_team = {"id": None}
zero_angles = {"m1": 0.0, "m2": 0.0}           # logical zero angles
calibration_offsets = {"m1": 0.0, "m2": 0.0}  # offsets from logical zero
current_target_team = {"id": None}            # target team we're aiming at

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
    time.sleep(1)
    GPIO.output(laser, GPIO.LOW)

# --- Calibration Functions ---
def save_zero(m1, m2):
    zero_angles["m1"] = m1.angle
    zero_angles["m2"] = m2.angle
    calibration_offsets["m1"] = -m1.angle
    calibration_offsets["m2"] = -m2.angle
    print("Saved zero angles (raw):", zero_angles)
    print("Saved calibration offsets:", calibration_offsets)

def return_to_zero(m1, m2):
    target_el = zero_angles["m1"] + calibration_offsets["m1"]
    target_az = zero_angles["m2"] + calibration_offsets["m2"]
    print(f"Returning to zero... targets: el={target_el:.3f}, az={target_az:.3f}")
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
        print("Team not found in JSON:", target_team)
        return
    st = self_team["id"]
    if st not in positions["turrets"]:
        print("ERROR: This turret's team number not in positions:", st)
        return

    # --- Store current target for display ---
    current_target_team["id"] = target_team

    # --- Positions in radians ---
    th_self = positions["turrets"][st]["theta"]
    th_tgt  = positions["turrets"][target_team]["theta"]

    # --- Cartesian positions (radius from JSON) ---
    r_self = positions["turrets"][st]["r"]
    r_tgt  = positions["turrets"][target_team]["r"]
    x_self = r_self * math.cos(th_self)
    y_self = r_self * math.sin(th_self)
    x_tgt  = r_tgt * math.cos(th_tgt)
    y_tgt  = r_tgt * math.sin(th_tgt)

    # --- Z positions ---
    z_self = turret_height_self
    z_tgt  = turret_height_other

    dx = x_tgt - x_self
    dy = y_tgt - y_self
    dz = z_tgt - z_self

    # --- Shortest azimuth angle (relative to current facing) ---
    az_target_rad = math.atan2(dy, dx)
    az_current_rad = th_self + math.pi  # turret points toward center initially
    delta_rad = (az_target_rad - az_current_rad + math.pi) % (2*math.pi) - math.pi
    az_deg = math.degrees(delta_rad)
    az_deg += calibration_offsets["m2"]

    # --- Elevation based on dz / horizontal distance ---
    horizontal_dist = math.sqrt(dx**2 + dy**2)
    el_rad = math.atan2(dz, horizontal_dist)
    el_deg = math.degrees(el_rad)
    el_deg += calibration_offsets["m1"]

    print(f"Aiming at team {target_team}: az={az_deg:.2f}°, el={el_deg:.2f}° | "
          f"horiz_dist={horizontal_dist:.2f} cm, dz={dz:.2f} cm")

    # --- Rotate motors ---
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

# --- Web Page with Half-Degree Jog and Current Teams ---
def web_page(m1_angle, m2_angle):
    def jog_buttons(name):
        buttons = [-90,-45,-15,-5,-1,-0.5,0.5,1,5,15,45,90]
        html_buttons = ""
        for b in buttons:
            html_buttons += f'<button name="{name}_jog" value="{b}">{b:+}°</button> '
        return html_buttons

    current_team = self_team["id"] if self_team["id"] else "N/A"
    target_team = current_target_team["id"] if current_target_team["id"] else "N/A"

    html = f"""
    <html>
    <head><title>Laser Turret</title></head>
    <body style="font-family: Arial; text-align:center; margin-top:20px;">
        <h2>Laser Turret Control</h2>
        <p>Current team: {current_team} | Aiming at: {target_team}</p>
        <form action="/" method="POST">
            <h3>Set This Turret's Team Number</h3>
            <input type="text" name="self_team" placeholder="Your team number"><br>
            <input type="submit" name="set_self_team" value="Set This Turret's Team"><br><br>

            <h3>Aim at Another Team</h3>
            <input type="text" name="team_box" placeholder="Target team #"><br>
            <input type="submit" name="aim_team" value="Aim at Team"><br><br>

            <h3>Manual Motor Control</h3>
            <h4>Azimuth</h4>
            <input type="text" name="m2" value="{m2_angle}"><br>
            {jog_buttons('m2')}
            <h4>Elevation</h4>
            <input type="text" name="m1" value="{m1_angle}"><br>
            {jog_buttons('m1')}<br><br>

            <input type="submit" value="Rotate Motors"><br><br>

            <h3>Calibration</h3>
            <input type="submit" name="return_zero" value="Return to Zero"><br>
            <input type="submit" name="save_zero" value="Save Current Position as Zero"><br><br>

            <h3>Laser</h3>
            <input type="submit" name="laser" value="Test Laser (1s)"><br>
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
            if "set_self_team" in data and data.get("self_team", "").strip():
                self_team["id"] = data["self_team"].strip()
                print("This turret's team set to:", self_team["id"])

            # Aim at another team
            if "aim_team" in data and data.get("team_box", "").strip():
                aim_at_team(m1, m2, data["team_box"].strip())

            # Manual motor control
            if "m1" in data and data["m1"].strip():
                try:
                    el = float(data["m1"])
                    p = m1.goAngle(el)
                    p.join()
                except: pass

            if "m2" in data and data["m2"].strip():
                try:
                    az = float(data["m2"])
                    p = m2.goAngle(az)
                    p.join()
                except: pass

            # Jog buttons
            for key in ["m1_jog","m2_jog"]:
                if key in data:
                    try:
                        delta = float(data[key])
                        p = m1.rotate(delta) if "m1" in key else m2.rotate(delta)
                        p.join()
                    except: pass

            # Calibration
            if "return_zero" in data: return_to_zero(m1, m2)
            if "save_zero" in data: save_zero(m1, m2)

            # Laser
            if "laser" in data: test_laser()

        try:
            response = web_page(m1.angle, m2.angle)
            conn.send(b"HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n\r\n")
            conn.sendall(response)
        except: pass

        conn.close()

# --- Main ---
if __name__ == "__main__":
    s = Shifter(23,24,25)
    lock1 = multiprocessing.Lock()
    lock2 = multiprocessing.Lock()
    m1 = Stepper(s, lock1, 0)  # elevation
    m2 = Stepper(s, lock2, 1)  # azimuth
    m1.zero()
    m2.zero()
    t = threading.Thread(target=serve_web, args=(m1,m2), daemon=True)
    t.start()
    try:
        while True: time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting.")
