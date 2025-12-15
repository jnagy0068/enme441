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
# Load positions from local JSON (UNCHANGED)
# --------------------------------------------------
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
            print("Available turret keys:",
                  list(positions.get("turrets", {}).keys()))
        except Exception as e:
            print("Error loading JSON:", e)
    else:
        print("JSON file not found at:", filename)

load_positions()

# --------------------------------------------------
# Turret State
# --------------------------------------------------
self_team = {"id": None}
current_target_team = {"id": None}

zero_angles = {"m1": 0.0, "m2": 0.0}
calibration_offsets = {"m1": 0.0, "m2": 0.0}

# --------------------------------------------------
# Stepper Motor Class (THREAD-BASED)
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
        self.angle = 0.0
        self.step_state = 0
        self.shifter_bit_start = 4 * index

    def _sgn(self, x):
        return 0 if x == 0 else int(abs(x) / x)

    def _step(self, direction):
        with self.lock:
            self.step_state = (self.step_state + direction) % 8

            my_bits = Stepper.seq[self.step_state] << self.shifter_bit_start
            final = self.s.last_value & ~(0b1111 << self.shifter_bit_start)
            final |= my_bits

            self.s.shiftByte(final)
            self.s.last_value = final

        self.angle = (self.angle +
                      direction / Stepper.steps_per_degree) % 360

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
        delta = (target - self.angle + 540) % 360 - 180
        return self.rotate(delta)

    def zero(self):
        self.angle = 0.0

# --------------------------------------------------
# Laser Functions
# --------------------------------------------------
def test_laser():
    GPIO.output(laser, GPIO.HIGH)
    time.sleep(1)
    GPIO.output(laser, GPIO.LOW)

# --------------------------------------------------
# Calibration Functions
# --------------------------------------------------
def save_zero(m1, m2):
    zero_angles["m1"] = m1.angle
    zero_angles["m2"] = m2.angle

    calibration_offsets["m1"] = -m1.angle
    calibration_offsets["m2"] = -m2.angle

    print("Saved zero angles:", zero_angles)
    print("Calibration offsets:", calibration_offsets)

def return_to_zero(m1, m2):
    el_target = zero_angles["m1"] + calibration_offsets["m1"]
    az_target = zero_angles["m2"] + calibration_offsets["m2"]

    print(f"Returning to zero: el={el_target:.2f}, az={az_target:.2f}")

    t1 = m1.goAngle(el_target)
    t2 = m2.goAngle(az_target)

    t1.join()
    t2.join()

# --------------------------------------------------
# Aim-at-Team Logic (UNCHANGED GEOMETRY)
# --------------------------------------------------
def aim_at_team(m1, m2, target_team):
    if self_team["id"] is None:
        print("ERROR: Self team not set.")
        return

    if target_team not in positions.get("turrets", {}):
        print("Target team not found:", target_team)
        return

    st = self_team["id"]
    if st not in positions["turrets"]:
        print("ERROR: Self team not in JSON:", st)
        return

    current_target_team["id"] = target_team

    r_self = positions["turrets"][st]["r"]
    theta_self = positions["turrets"][st]["theta"]
    r_tgt = positions["turrets"][target_team]["r"]
    theta_tgt = positions["turrets"][target_team]["theta"]

    x_self = r_self * math.cos(theta_self)
    y_self = r_self * math.sin(theta_self)
    x_tgt = r_tgt * math.cos(theta_tgt)
    y_tgt = r_tgt * math.sin(theta_tgt)

    z_self = turret_height_self
    z_tgt = turret_height_other

    dx = x_tgt - x_self
    dy = y_tgt - y_self
    dz = z_tgt - z_self

    az_to_target = math.atan2(dy, dx)
    az_current = theta_self + math.pi
    delta_az = (az_to_target - az_current + math.pi) % (2 * math.pi) - math.pi
    az_deg = math.degrees(delta_az) + calibration_offsets["m2"]

    horiz_dist = math.hypot(dx, dy)
    el_rad = math.atan2(dz, horiz_dist)
    el_deg = math.degrees(el_rad) + calibration_offsets["m1"]

    print(f"Aiming at team {target_team}: "
          f"az={az_deg:.2f}°, el={el_deg:.2f}°")

    t1 = m1.goAngle(el_deg)
    t2 = m2.goAngle(az_deg)

    t1.join()
    t2.join()

# --------------------------------------------------
# POST Parsing
# --------------------------------------------------
def parsePOSTdata(data):
    idx = data.find("\r\n\r\n")
    if idx == -1:
        return {}
    post = data[idx + 4:]
    parsed = parse_qs(post, keep_blank_values=True)
    return {k: v[0] for k, v in parsed.items()}

# --------------------------------------------------
# Web Page
# --------------------------------------------------
def web_page(m1_angle, m2_angle):
    def jog_buttons(name):
        buttons = [-90,-45,-15,-5,-1,-0.5,0.5,1,5,15,45,90]
        return " ".join(
            f'<button name="{name}_jog" value="{b}">{b:+}°</button>'
            for b in buttons
        )

    current_team = self_team["id"] or "N/A"
    target_team = current_target_team["id"] or "N/A"

    html = f"""
    <html>
    <head><title>Laser Turret</title></head>
    <body style="font-family:Arial;text-align:center;margin-top:20px;">
        <h2>Laser Turret Control</h2>
        <p>Current team: {current_team} | Aiming at: {target_team}</p>

        <form action="/" method="POST">
            <h3>Set This Turret's Team</h3>
            <input type="text" name="self_team">
            <input type="submit" name="set_self_team" value="Set"><br><br>

            <h3>Aim at Team</h3>
            <input type="text" name="team_box">
            <input type="submit" name="aim_team" value="Aim"><br><br>

            <h3>Manual Control</h3>
            <h4>Azimuth</h4>
            <input type="text" name="m2" value="{m2_angle:.2f}"><br>
            {jog_buttons('m2')}

            <h4>Elevation</h4>
            <input type="text" name="m1" value="{m1_angle:.2f}"><br>
            {jog_buttons('m1')}<br><br>

            <input type="submit" value="Rotate"><br><br>

            <h3>Calibration</h3>
            <input type="submit" name="return_zero" value="Return to Zero">
            <input type="submit" name="save_zero" value="Save Zero"><br><br>

            <h3>Laser</h3>
            <input type="submit" name="laser" value="Test Laser">
        </form>
    </body>
    </html>
    """
    return html.encode("utf-8")

# --------------------------------------------------
# Web Server
# --------------------------------------------------
def serve_web(m1, m2):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('', 8080))
    s.listen(3)

    print("Web server running on port 8080")

    while True:
        conn, _ = s.accept()
        msg = conn.recv(4096).decode(errors="ignore")

        if msg.startswith("POST"):
            data = parsePOSTdata(msg)

            if "set_self_team" in data and data.get("self_team"):
                self_team["id"] = data["self_team"].strip()
                print("Self team set to:", self_team["id"])

            if "aim_team" in data and data.get("team_box"):
                aim_at_team(m1, m2, data["team_box"].strip())

            if "m1" in data and data["m1"]:
                try:
                    m1.goAngle(float(data["m1"])).join()
                except:
                    pass

            if "m2" in data and data["m2"]:
                try:
                    m2.goAngle(float(data["m2"])).join()
                except:
                    pass

            for key in ("m1_jog", "m2_jog"):
                if key in data:
                    try:
                        delta = float(data[key])
                        (m1 if "m1" in key else m2).rotate(delta).join()
                    except:
                        pass

            if "return_zero" in data:
                return_to_zero(m1, m2)

            if "save_zero" in data:
                save_zero(m1, m2)

            if "laser" in data:
                test_laser()

        conn.send(b"HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n")
        conn.sendall(web_page(m1.angle, m2.angle))
        conn.close()

# --------------------------------------------------
# Main
# --------------------------------------------------
if __name__ == "__main__":
    shifter = Shifter(23, 24, 25)
    shifter.last_value = 0

    shared_lock = threading.Lock()

    m1 = Stepper(shifter, shared_lock, 0)  # elevation
    m2 = Stepper(shifter, shared_lock, 1)  # azimuth

    m1.zero()
    m2.zero()

    threading.Thread(target=serve_web, args=(m1, m2),
                     daemon=True).start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        GPIO.cleanup()
        print("Exiting.")
