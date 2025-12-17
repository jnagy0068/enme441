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
turret_height_self = 34.0     # your turret laser height (cm)
turret_height_other = 3.0    # all other turrets' laser height (cm)

# --- GPIO Setup ---
GPIO.setmode(GPIO.BCM)
laser = 22
GPIO.setup(laser, GPIO.OUT)
GPIO.output(laser, GPIO.LOW)

# --- Shared Memory for Shifter bits (still fine for threading) ---
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

#  Turret and Calibration Data -
calibration = {"az_offset": 0.0, "el_offset": 0.0}
self_team = {"id": None}

# New: store absolute motor angles that correspond to your *physical zero*
zero_positions = {"m1": 0.0, "m2": 0.0}

def normalize_deg(angle):
    """Normalize angle to [-180, 180)."""
    a = ((angle + 180.0) % 360.0) - 180.0
    # convert -180.0 to +180.0 consistently if desired (keep -180)
    return a

# --- Stepper Motor Class (uses threads so .angle is accurate in main process) ---
class Stepper:
    seq = [0b0001,0b0011,0b0010,0b0110,0b0100,0b1100,0b1000,0b1001]
    delay = 500                         # microsecond delay used previously
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
        with self.lock:
            self.step_state = (self.step_state + direction) % 8
            myArray[self.index] &= ~(0b1111 << self.shifter_bit_start)
            myArray[self.index] |= (Stepper.seq[self.step_state] << self.shifter_bit_start)
            final = 0
            for val in myArray:
                final |= val
            self.s.shiftByte(final)
        # update angle in same process (degrees)
        self.angle = (self.angle + direction/Stepper.steps_per_degree) % 360.0
        time.sleep(Stepper.delay / 1e6)

    def _rotate(self, delta):
        direction = self._sgn(delta)
        steps = int(abs(delta) * Stepper.steps_per_degree)
        for _ in range(steps):
            self._step(direction)

    # start a thread (not a process) so angle stays updated in parent
    def rotate(self, delta):
        t = threading.Thread(target=self._rotate, args=(delta,), daemon=True)
        t.start()
        self._thread = t
        return t

    def goAngle(self, target):
        # compute smallest rotation to target (target is absolute motor angle)
        # normalize target to [-180,180) internal representation optional
        # delta computed so motor moves shortest path
        delta = (target - self.angle + 540.0) % 360.0 - 180.0
        return self.rotate(delta)

    def zero(self):
        self.angle = 0.0

# --- Laser Function ---
def test_laser():
    GPIO.output(laser, GPIO.HIGH)
    time.sleep(1)
    GPIO.output(laser, GPIO.LOW)

# --- Calibration & Zero Functions ---
def save_zero(m1, m2):
    """
    Save the *current absolute motor angles* as the physical zero.
    Also print a normalized and raw view for debugging.
    """
    zero_positions["m1"] = m1.angle
    zero_positions["m2"] = m2.angle

    # Keep calibration offsets untouched; calibration is for aiming bias corrections
    print("Saved zero_positions (raw):", {"m1": m1.angle, "m2": m2.angle})
    print("Saved zero_positions (normalized):", {"m1": normalize_deg(m1.angle), "m2": normalize_deg(m2.angle)})

def return_to_zero(m1, m2):
    """
    Return motors to the saved absolute angles in zero_positions.
    These are absolute motor angles (not negatives).
    """
    tgt_m1 = zero_positions.get("m1", 0.0)
    tgt_m2 = zero_positions.get("m2", 0.0)
    print("Returning to zero... targets (raw):", {"m1": tgt_m1, "m2": tgt_m2})
    print("Returning to zero... targets (norm):", {"m1": normalize_deg(tgt_m1), "m2": normalize_deg(tgt_m2)})

    p1 = m1.goAngle(tgt_m1)
    p2 = m2.goAngle(tgt_m2)
    p1.join()
    p2.join()

# --- Correct Aim at Team for Circumference with center-zero elevation compensation ---
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

    # read r and theta from JSON (assume units consistent e.g. cm)
    r_self  = float(positions["turrets"][st]["r"])
    th_self = float(positions["turrets"][st]["theta"])
    r_tgt   = float(positions["turrets"][target_team]["r"])
    th_tgt  = float(positions["turrets"][target_team]["theta"])

    # Cartesian positions in same unit (cm)
    x_self = r_self * math.cos(th_self)
    y_self = r_self * math.sin(th_self)
    x_tgt  = r_tgt  * math.cos(th_tgt)
    y_tgt  = r_tgt  * math.sin(th_tgt)

    # heights (cm)
    z_self = float(turret_height_self)
    z_tgt  = float(turret_height_other)

    # vector from turret to target
    dx = x_tgt - x_self
    dy = y_tgt - y_self
    dz = z_tgt - z_self

    horizontal_dist = math.hypot(dx, dy)  # ground-plane distance (cm)

    # Absolute elevation (wrt horizontal plane) to the target
    el_target_abs = math.atan2(dz, horizontal_dist)   # radians

    # Absolute elevation to the center (this is your physical zero)
    dz_center = 0.0 - z_self
    dist_to_center = r_self
    el_center_abs = math.atan2(dz_center, dist_to_center)  # radians

    # Elevation command = difference from center-zero (in degrees)
    el_rel_rad = el_target_abs - el_center_abs
    el_deg = el_rel_rad * 180.0 / math.pi

    # Azimuth: absolute azimuth to target in world coords
    az_world = math.atan2(dy, dx)
    turret_facing = th_self + math.pi   # turret zero points to center
    az_rel = az_world - turret_facing
    az_deg = -az_rel * 180.0 / math.pi  # NEGATE to match manual controls convention

    # normalize azimuth to [-180, 180)
    az_deg = normalize_deg(az_deg)

    # Apply calibration offsets saved for aiming bias (not zero positions)
    az_deg += calibration.get("az_offset", 0.0)
    el_deg += calibration.get("el_offset", 0.0)

    print(
        f"Aiming at team {target_team}: az={az_deg:.2f}°, el={el_deg:.2f}° | "
        f"horiz_dist={horizontal_dist:.2f} cm, dz={dz:.2f} cm, "
        f"el_target_abs={el_target_abs*180/math.pi:.4f}°, el_center_abs={el_center_abs*180/math.pi:.4f}°"
    )

    # target absolute angles for motors:
    # elevation motor target = zero_positions['m1'] + el_deg
    # azimuth motor target = zero_positions['m2'] + az_deg
    tgt_el_abs = zero_positions.get("m1", 0.0) + el_deg
    tgt_az_abs = zero_positions.get("m2", 0.0) + az_deg

    # Normalize targets into [0, 360) or keep raw; Stepper.goAngle handles shortest path
    # but for readability we normalize for print
    print("Computed motor targets (raw):", {"el": tgt_el_abs, "az": tgt_az_abs})
    print("Computed motor targets (norm):", {"el": normalize_deg(tgt_el_abs), "az": normalize_deg(tgt_az_abs)})

    # Rotate motors (elevation = m1, azimuth = m2)
    p1 = m1.goAngle(tgt_el_abs)
    p2 = m2.goAngle(tgt_az_abs)
    p1.join()
    p2.join()

def aim_and_laser_sequence(m1, m2):
    if self_team["id"] is None:
        print("ERROR: Self team not set.")
        return

    print("Starting full aim + laser sequence (sorted by theta)")

    # ---- Turrets (sorted by theta) ----
    turret_items = [
        (tid, data)
        for tid, data in positions.get("turrets", {}).items()
        if tid != self_team["id"]
    ]

    turret_items.sort(key=lambda item: float(item[1]["theta"]))

    for team_id, _ in turret_items:
        print("Targeting turret:", team_id)
        aim_at_team(m1, m2, team_id)

        GPIO.output(laser, GPIO.HIGH)
        time.sleep(3)
        GPIO.output(laser, GPIO.LOW)

        time.sleep(0.5)

    # ---- Globes (sorted by theta) ----
    globe_items = list(positions.get("globes", {}).items())
    globe_items.sort(key=lambda item: float(item[1]["theta"]))

    for globe_id, globe_data in globe_items:
        print("Targeting globe:", globe_id)

        # temporarily inject globe as a turret target
        positions["turrets"]["__globe__"] = globe_data
        aim_at_team(m1, m2, "__globe__")
        del positions["turrets"]["__globe__"]

        GPIO.output(laser, GPIO.HIGH)
        time.sleep(3)
        GPIO.output(laser, GPIO.LOW)

        time.sleep(0.5)

    print("Sequence complete")

# --- POST Parsing ---
def parsePOSTdata(data):
    idx = data.find("\r\n\r\n")
    if idx == -1:
        return {}
    post = data[idx+4:]
    parsed = parse_qs(post, keep_blank_values=True)
    return {k:v[0] for k,v in parsed.items()}

# --- Web Page with Manual Jog Buttons and current-angle display ---
def web_page(m1_angle, m2_angle):
    def jog_buttons(name):
        buttons = [-45, -15, -5, -1, -0.5, -0.1, 0.1, 0.5, 1, 5, 15, 45]
        html_buttons = ""
        for b in buttons:
            html_buttons += f'<button name="{name}_jog" value="{b}">{b:+}°</button> '
        return html_buttons

    html = f"""
    <html>
    <head><title>Laser Turret</title></head>
    <body style="font-family: Arial; text-align:center; margin-top:20px;">
        <h2>Laser Turret Control</h2>

        <div>
            <strong>Current angles:</strong>
            <div>Elevation (m1): {m1_angle:.2f}°</div>
            <div>Azimuth (m2): {m2_angle:.2f}°</div>
        </div>
        <hr>

        <form action="/" method="POST">
            <h3>Team Number</h3>
            <input type="text" name="self_team" placeholder="Team number"><br>
            <input type="submit" name="set_self_team" value="Set This Turret's Team"><br>

            <h3>Aim at Another Team</h3>
            <input type="text" name="team_box" placeholder="Target team #"><br>
            <input type="submit" name="aim_team" value="Aim at Team"><br>

            <h3>Manual Motor Control</h3>
            <h4>Azimuth</h4>
            {jog_buttons('m2')}
            <h4>Elevation</h4>
            {jog_buttons('m1')}

            <br><input type="submit" value="Rotate Motors"><br>

            <h3>Calibration</h3>
            <input type="submit" name="return_zero" value="Return to Zero"><br>
            <input type="submit" name="save_zero" value="Save Current Position as Zero"><br>

            <h3>Laser</h3>
            <input type="submit" name="laser" value="Test Laser (1s)"><br>
            <h3>Automatic Sequence</h3>
            <input type="submit" name="auto_sequence" value="Fire!">
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
            msg = conn.recv(8192).decode(errors='ignore')
        except:
            conn.close()
            continue

        if msg.startswith("POST"):
            data = parsePOSTdata(msg)
        
            # -----------------------------
            # 1. Set self team
            # -----------------------------
            if "set_self_team" in data and data.get("self_team", "").strip() != "":
                self_team["id"] = data["self_team"].strip()
                print("This turret's team set to:", self_team["id"])
        
            # -----------------------------
            # 2. Aim at another team
            # -----------------------------
            if "aim_team" in data and data.get("team_box", "").strip() != "":
                aim_at_team(m1, m2, data["team_box"].strip())
        
            # -----------------------------
            # 3. Jog buttons (rotate by delta)
            # -----------------------------
            if "m1_jog" in data:
                try:
                    delta = float(data["m1_jog"])
                    p = m1.rotate(delta)
                    p.join()
                except:
                    pass
        
            if "m2_jog" in data:
                try:
                    delta = float(data["m2_jog"])
                    p = m2.rotate(delta)
                    p.join()
                except:
                    pass
        
            # -----------------------------
            # 4. Manual absolute control ONLY when clicking "Rotate Motors"
            # -----------------------------
            if any(v == "Rotate Motors" for v in data.values()):
                # elevation setpoint
                if "m1" in data and data["m1"].strip() != "":
                    try:
                        el = float(data["m1"])
                        p = m1.goAngle(el)
                        p.join()
                    except:
                        pass
        
                # azimuth setpoint
                if "m2" in data and data["m2"].strip() != "":
                    try:
                        az = float(data["m2"])
                        p = m2.goAngle(az)
                        p.join()
                    except:
                        pass
        
            # -----------------------------
            # 5. Calibration
            # -----------------------------
            if "return_zero" in data:
                return_to_zero(m1, m2)
        
            if "save_zero" in data:
                save_zero(m1, m2)
        
            # -----------------------------
            # 6. Laser test (NO motor movement)
            # -----------------------------
            if "laser" in data:
                test_laser()
            # -----------------------------
            # 7. Automatic aim + laser sequence
            # -----------------------------
            if "auto_sequence" in data:
                t = threading.Thread(
                    target=aim_and_laser_sequence,
                    args=(m1, m2),
                    daemon=True
                )
                t.start()
        try:
            response = web_page(m1.angle, m2.angle)
            conn.send(b"HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n\r\n")
            conn.sendall(response)
        except Exception as e:
            print("Failed sending response:", e)

        conn.close()

# --- Main ---
if __name__ == "__main__":
    s = Shifter(23, 24, 25)
    lock1 = threading.Lock()
    lock2 = threading.Lock()

    m1 = Stepper(s, lock1, 0)  # elevation
    m2 = Stepper(s, lock2, 1)  # azimuth

    # initialize zero_positions to current motor angles (whatever motors start at)
    zero_positions["m1"] = m1.angle
    zero_positions["m2"] = m2.angle
    print("Initial zero_positions (raw):", zero_positions)

    t = threading.Thread(target=serve_web, args=(m1, m2), daemon=True)
    t.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting.")
