# project(1).py  -- modified to load positions.json from URL or (commented) local file,
# add calibration button, team selection dropdown, and automated targeting sequence.
import time
import multiprocessing
import socket
import threading
from shifter import Shifter
import RPi.GPIO as GPIO
import urllib.request
import json
import math
import os

GPIO.setmode(GPIO.BCM)
LASER_PIN = 22
GPIO.setup(LASER_PIN, GPIO.OUT)
GPIO.output(LASER_PIN, GPIO.LOW)

myArray = multiprocessing.Array('i', 2)

# -------------------------
# Stepper motor abstraction
# -------------------------
class Stepper:
    seq = [0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001]
    delay = 500  # microseconds per step
    # steps_per_degree based on your mechanical setup (keep existing value)
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
        # run rotation in a separate process so web interface stays responsive
        p = multiprocessing.Process(target=self._rotate, args=(delta,))
        p.start()
        return p

    def goAngle(self, target):
        # choose shortest rotation
        delta = (target - self.angle + 540) % 360 - 180
        return self.rotate(delta)

    def zero(self):
        self.angle = 0.0

# -------------------------
# Laser helper
# -------------------------
def test_laser(duration=3.0):
    GPIO.output(LASER_PIN, GPIO.HIGH)
    time.sleep(duration)
    GPIO.output(LASER_PIN, GPIO.LOW)

# -------------------------
# Data & calibration state
# -------------------------
positions = {}   # loaded positions JSON
calibration = {"m1_offset": 0.0, "m2_offset": 0.0}
calibration_file = "calibration.json"
# load calibration if exists
if os.path.exists(calibration_file):
    try:
        with open(calibration_file, 'r') as f:
            calibration = json.load(f)
    except:
        calibration = {"m1_offset": 0.0, "m2_offset": 0.0}

# Default JSON URL (as you specified)
DEFAULT_JSON_URL = "http://192.168.1.254:8000/positions.json"

# -------------------------
# Position loading
# -------------------------
def fetch_positions_from_url(url):
    global positions
    try:
        with urllib.request.urlopen(url, timeout=5) as resp:
            data = resp.read().decode('utf-8')
            positions = json.loads(data)
            return True, "Loaded positions from URL."
    except Exception as e:
        return False, f"Failed to load from URL: {e}"

def load_positions_from_local(path="positions.json"):
    # This function is provided for local-file testing.
    # The call below in serve_web remains commented-out as requested.
    global positions
    try:
        with open(path, 'r') as f:
            positions = json.load(f)
            return True, "Loaded positions from local file."
    except Exception as e:
        return False, f"Failed to load local file: {e}"

# -------------------------
# Geometry helpers
# -------------------------
def polar_to_cartesian(r, theta):
    # JSON theta is in radians; returns (x,y)
    x = r * math.cos(theta)
    y = r * math.sin(theta)
    return x, y

def compute_aim_angles(self_r, self_theta, target):
    """
    target: dict with keys r, theta, optionally z
    returns: (azimuth_deg, elevation_deg)
    - azimuth is degrees the turret must rotate to point at the target (0..360)
    - elevation is degrees above horizontal (0 = horizontal, positive up)
    """

    tx, ty = polar_to_cartesian(target["r"], target["theta"])
    sx, sy = polar_to_cartesian(self_r, self_theta)

    # vector from self to target in field coords (cm)
    dx = tx - sx
    dy = ty - sy
    ground_dist = math.hypot(dx, dy)  # horizontal distance (cm)

    # azimuth relative to turret forward (we assume turret's 0 deg aligns with +x axis in stepper angle convention)
    azimuth_rad = math.atan2(dy, dx)
    azimuth_deg = math.degrees(azimuth_rad) % 360

    # elevation: if target has z (cm) use it; otherwise assume target approx same height -> z=0
    z = target.get("z", 0.0)
    # guard against zero distance
    if ground_dist == 0 and z == 0:
        elevation_deg = 0.0
    else:
        elevation_rad = math.atan2(z, ground_dist)
        elevation_deg = math.degrees(elevation_rad)

    return azimuth_deg, elevation_deg

# -------------------------
# Web UI generation
# -------------------------
def web_page(m1_angle, m2_angle, status_msg="", loaded_url=DEFAULT_JSON_URL, selected_team=""):
    # Build team options from loaded positions if available
    team_options_html = ""
    turrets_obj = positions.get("turrets", {})
    sorted_keys = sorted([k for k in turrets_obj.keys()], key=lambda x: int(x) if x.isdigit() else x)
    for k in sorted_keys:
        sel = "selected" if str(k) == str(selected_team) else ""
        team_options_html += f'<option value="{k}" {sel}>Team {k}</option>\n'

    html = f"""
    <html>
    <head><title>Stepper Control</title></head>
    <body style="font-family: Arial; text-align:center; margin-top:20px;">
        <h2>Stepper Motor & Turret Control</h2>
        <p style="color:green;">{status_msg}</p>

        <form action="/" method="POST">
            <label>Motor 1 Angle (degrees):</label><br>
            <input type="text" name="m1" value="{m1_angle}"><br><br>

            <label>Motor 2 Angle (degrees):</label><br>
            <input type="text" name="m2" value="{m2_angle}"><br><br>

            <input type="submit" value="Rotate Motors"><br><br>

            <input type="submit" name="laser" value="Test Laser (3s)">
            <hr style="width:60%;">

            <label>Positions JSON URL:</label><br>
            <input type="text" name="json_url" value="{loaded_url}" style="width:60%;"><br><br>
            <input type="submit" name="load_positions" value="Load Positions from URL">
            <!-- To load a local positions.json instead, uncomment local load code in the server (see script). -->

            <br><br>

            <label>Select Team (or type number):</label><br>
            <select name="team_select">
                <option value="">-- Choose team --</option>
                {team_options_html}
            </select>
            <input type="text" name="team_manual" placeholder="Or enter team number" value="{selected_team}"><br><br>

            <input type="submit" name="calibrate" value="Calibrate (set current position as 0,0)"><br><br>

            <input type="submit" name="auto_target" value="Start Auto-Targeting Sequence"><br><br>

            <input type="submit" name="stop_auto" value="Stop Auto (useful to interrupt)"><br><br>

            <hr style="width:60%;">
            <p>Calibration offsets: M1 = {calibration.get('m1_offset',0):.2f}°, M2 = {calibration.get('m2_offset',0):.2f}°</p>
        </form>
    </body>
    </html>
    """
    return bytes(html, 'utf-8')

# -------------------------
# Auto-targeting controller
# -------------------------
auto_targeting_flag = multiprocessing.Value('i', 0)  # 0 = idle, 1 = running

def run_target_sequence(m1, m2, team_num):
    """
    This runs in a separate process. It:
      - determines self turret position from loaded positions
      - builds a list of targets (all other turrets + globes)
      - for each target: compute azimuth & elevation, compensate calibration offsets, aim, and fire for 3s
    """
    with auto_targeting_flag.get_lock():
        if auto_targeting_flag.value == 1:
            # already running
            return
        auto_targeting_flag.value = 1

    try:
        turrets = positions.get("turrets", {})
        globes = positions.get("globes", [])

        if str(team_num) not in turrets:
            print("Team not found in positions; aborting auto-target.")
            return

        self_pos = turrets[str(team_num)]
        self_r = float(self_pos["r"])
        self_theta = float(self_pos["theta"])

        # create target list: other turrets (z=0) and globes
        target_list = []
        for k, v in turrets.items():
            if str(k) == str(team_num):
                continue
            target_list.append({"r": float(v["r"]), "theta": float(v["theta"]), "label": f"turret_{k}", "z": 0.0})

        for g in globes:
            target_list.append({"r": float(g["r"]), "theta": float(g["theta"]), "z": float(g.get("z", 0.0)), "label": "globe"})

        for tgt in target_list:
            with auto_targeting_flag.get_lock():
                if auto_targeting_flag.value == 0:
                    print("Auto-targeting interrupted.")
                    return

            azimuth_deg, elevation_deg = compute_aim_angles(self_r, self_theta, tgt)

            # compensate calibration offsets (we stored offsets so that "zero" point maps correctly)
            azimuth_deg = (azimuth_deg + calibration.get("m1_offset", 0.0)) % 360
            elevation_deg = (elevation_deg + calibration.get("m2_offset", 0.0)) % 360

            print(f"Aiming at {tgt.get('label','target')}: az={azimuth_deg:.2f}°, el={elevation_deg:.2f}°")
            # Move motors (wait for both to finish)
            p1 = m1.goAngle(azimuth_deg)
            p2 = m2.goAngle(elevation_deg)
            p1.join()
            p2.join()

            # fire laser for 3 seconds
            test_laser(3.0)
            time.sleep(0.5)  # brief pause between targets

    finally:
        with auto_targeting_flag.get_lock():
            auto_targeting_flag.value = 0

# -------------------------
# Helper: parse POST data
# -------------------------
def parsePOSTdata(data):
    data_dict = {}
    idx = data.find('\r\n\r\n') + 4
    post = data[idx:]
    pairs = post.split('&')
    for p in pairs:
        if '=' in p:
            key, val = p.split('=')
            data_dict[key] = urllib.parse.unquote_plus(val)
    return data_dict

# -------------------------
# Web server
# -------------------------
def serve_web(m1, m2):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('', 8080))
    s.listen(3)
    print("Web server running on port 8080...")

    status_msg = ""
    loaded_url = DEFAULT_JSON_URL
    selected_team = ""

    while True:
        conn, addr = s.accept()
        msg = conn.recv(4096).decode(errors='ignore')

        m1_target = ""
        m2_target = ""

        if msg.startswith("POST"):
            data = parsePOSTdata(msg)

            # rotate motors manually
            if "m1" in data and data["m1"].strip() != "":
                try:
                    m1_target = float(data["m1"])
                    p = m1.goAngle(m1_target)
                    p.join()
                    status_msg = f"Motor 1 rotated to {m1_target}°"
                except Exception as e:
                    status_msg = f"Error rotating M1: {e}"

            if "m2" in data and data["m2"].strip() != "":
                try:
                    m2_target = float(data["m2"])
                    p = m2.goAngle(m2_target)
                    p.join()
                    status_msg = f"Motor 2 rotated to {m2_target}°"
                except Exception as e:
                    status_msg = f"Error rotating M2: {e}"

            # laser test
            if "laser" in data:
                test_laser()
                status_msg = "Laser test executed."

            # load positions from URL
            if "load_positions" in data:
                loaded_url = data.get("json_url", DEFAULT_JSON_URL)
                ok, msg_text = fetch_positions_from_url(loaded_url)
                status_msg = msg_text
                # For local testing instead of fetching from URL, you can uncomment:
                # ok, msg_text = load_positions_from_local('/mnt/data/test_positions.json')
                # status_msg = msg_text

            # team selection: prefer manual typed number over dropdown blank
            team_selected = data.get("team_manual", "").strip()
            if team_selected == "":
                team_selected = data.get("team_select", "").strip()
            selected_team = team_selected

            # calibrate (store current motor angles as offsets so that current pointing becomes 0,0)
            if "calibrate" in data:
                # We want subsequent target computations to treat current absolute motor angles as the zero reference.
                calibration["m1_offset"] = (-m1.angle) % 360
                calibration["m2_offset"] = (-m2.angle) % 360
                try:
                    with open(calibration_file, 'w') as f:
                        json.dump(calibration, f)
                    status_msg = f"Calibrated. Offsets saved (m1={calibration['m1_offset']:.2f}, m2={calibration['m2_offset']:.2f})."
                except Exception as e:
                    status_msg = f"Calibration saved failed: {e}"

            # auto-targeting
            if "auto_target" in data:
                if selected_team == "" or positions.get("turrets", {}) == {}:
                    status_msg = "Load positions and choose a team before running auto-target."
                else:
                    # start auto-target in a separate process
                    p = multiprocessing.Process(target=run_target_sequence, args=(m1, m2, selected_team))
                    p.start()
                    status_msg = "Auto-target sequence started."

            # stop auto-target
            if "stop_auto" in data:
                with auto_targeting_flag.get_lock():
                    auto_targeting_flag.value = 0
                status_msg = "Auto-target stop requested."

        # Build response
        response = web_page(m1.angle, m2.angle, status_msg=status_msg, loaded_url=loaded_url, selected_team=selected_team)
        try:
            conn.send(b'HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n')
            conn.sendall(response)
        except Exception as e:
            print("Error sending response:", e)
        conn.close()

# -------------------------
# Main init
# -------------------------
if __name__ == '__main__':
    s = Shifter(23, 24, 25)
    lock1 = multiprocessing.Lock()
    lock2 = multiprocessing.Lock()

    m1 = Stepper(s, lock1, 0)  # azimuth
    m2 = Stepper(s, lock2, 1)  # elevation

    m1.zero()
    m2.zero()

    # Optionally preload test positions for debugging
    # You can enable local load for testing on your Pi by uncommenting the next line:
    # ok,msg = load_positions_from_local('/mnt/data/test_positions.json'); print(msg)

    # Start web server thread
    t = threading.Thread(target=serve_web, args=(m1, m2), daemon=True)
    t.start()

    print("Motors initialized. Web interface ready.")
    print("Open a browser to: http://<raspberry-pi-ip>:8080")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting.")
