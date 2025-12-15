import time
import socket
import threading
import json
import math
import os
from urllib.parse import parse_qs
import logging
import queue

from shifter import Shifter
import RPi.GPIO as GPIO

# ---------------------- Logging ----------------------
logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')

# ---------------------- Turret Heights ----------------------
turret_height_self = 9.0
turret_height_other = 0.0

# ---------------------- GPIO ----------------------
GPIO.setmode(GPIO.BCM)
laser = 22
GPIO.setup(laser, GPIO.OUT)
GPIO.output(laser, GPIO.LOW)

# ---------------------- Load Positions ----------------------
positions = {}

def load_positions():
    global positions
    script_dir = os.path.dirname(os.path.realpath(__file__))
    filename = os.path.join(script_dir, "test_positions.json")
    if os.path.exists(filename):
        with open(filename, "r") as f:
            positions.update(json.load(f))
        logging.info("Loaded positions JSON.")
    else:
        logging.warning("JSON file not found.")

load_positions()

# ---------------------- Turret State ----------------------
self_team = {"id": None}
current_target_team = {"id": None}

zero_angles = {"m1": 0.0, "m2": 0.0}
calibration_offsets = {"m1": 0.0, "m2": 0.0}

# ---------------------- Stepper ----------------------
class Stepper:
    seq = [0b0001,0b0011,0b0010,0b0110,
           0b0100,0b1100,0b1000,0b1001]
    delay = 0.001  # seconds
    steps_per_degree = 4 * 1024 / 360

    def __init__(self, shifter, lock, index):
        self.s = shifter
        self.lock = lock
        self.index = index
        self.angle = 0.0
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
        self.angle = (self.angle + direction / Stepper.steps_per_degree) % 360
        time.sleep(Stepper.delay)

    def rotate(self, delta):
        direction = self._sgn(delta)
        steps = int(abs(delta) * Stepper.steps_per_degree)
        for _ in range(steps):
            self._step(direction)

    def goAngle(self, target):
        delta = (target - self.angle + 540) % 360 - 180
        self.rotate(delta)

    def zero(self):
        self.angle = 0.0

# ---------------------- Motor Worker ----------------------
class MotorWorker(threading.Thread):
    def __init__(self, motor):
        super().__init__(daemon=True)
        self.motor = motor
        self.commands = queue.Queue()

    def run(self):
        while True:
            delta = self.commands.get()
            if delta is None:
                break
            try:
                self.motor.rotate(delta)
            except Exception as e:
                logging.error(f"Motor error: {e}")
            self.commands.task_done()

    def rotate(self, delta):
        self.commands.put(delta)

# ---------------------- Laser ----------------------
def test_laser():
    GPIO.output(laser, GPIO.HIGH)
    time.sleep(1)
    GPIO.output(laser, GPIO.LOW)

# ---------------------- Calibration ----------------------
def save_zero(m1, m2):
    zero_angles["m1"] = m1.angle
    zero_angles["m2"] = m2.angle
    calibration_offsets["m1"] = -m1.angle
    calibration_offsets["m2"] = -m2.angle
    logging.info("Calibration saved.")

def return_to_zero(m1, m2):
    m1_delta = zero_angles["m1"] + calibration_offsets["m1"] - m1.angle
    m2_delta = zero_angles["m2"] + calibration_offsets["m2"] - m2.angle
    m1_worker.rotate(m1_delta)
    m2_worker.rotate(m2_delta)
    logging.info("Returning to zero.")

# ---------------------- Aim at Team ----------------------
def aim_at_team(m1_worker, m2_worker, target_team):
    if self_team["id"] is None:
        logging.warning("Self team not set; cannot aim.")
        return
    if target_team not in positions.get("turrets", {}):
        logging.warning(f"Target team {target_team} not found.")
        return
    st = self_team["id"]
    if st not in positions["turrets"]:
        logging.warning(f"Self team {st} not found.")
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
    dz = turret_height_other - turret_height_self

    az_deg = math.degrees(math.atan2(dy, dx)) + calibration_offsets["m2"]
    el_deg = -math.degrees(math.atan2(dz, math.hypot(dx, dy))) + calibration_offsets["m1"]

    delta_m1 = el_deg - m1_worker.motor.angle
    delta_m2 = az_deg - m2_worker.motor.angle

    m1_worker.rotate(delta_m1)
    m2_worker.rotate(delta_m2)

    logging.info(f"Aiming at team {target_team}: az={az_deg:.2f}, el={el_deg:.2f}")

# ---------------------- POST Parsing ----------------------
def parsePOSTdata(data):
    idx = data.find("\r\n\r\n")
    if idx == -1:
        return {}
    parsed = parse_qs(data[idx+4:], keep_blank_values=True)
    return {k:v[0] for k,v in parsed.items()}

# ---------------------- Web Page ----------------------
def web_page(m1_angle, m2_angle):
    def jog_buttons(name):
        vals = [-90, -45, -15, -5, -1, -0.5, 0.5, 1, 5, 15, 45, 90]
        return " ".join(f'<button name="{name}_jog" value="{v}">{v:+}°</button>' for v in vals)

    self_id = self_team.get('id', 'N/A')
    target_id = current_target_team.get('id', 'N/A')

    return f"""
<html>
<head>
<title>Laser Turret Control</title>
<style>
body {{ font-family: Arial; text-align:center; background:#f4f4f4; }}
.section {{ background:#fff; padding:15px; margin:10px auto; border-radius:10px; width:400px; box-shadow:2px 2px 10px #aaa; }}
button {{ margin:2px; padding:5px 10px; }}
input[type="text"] {{ width:80px; text-align:center; }}
</style>
</head>
<body>
<h2>Laser Turret Control</h2>
<div class="section">
<p><strong>Self Team:</strong> {self_id} | <strong>Target:</strong> {target_id}</p>
</div>
<div class="section">
<form method="POST">
<h3>Set Team</h3>
<input name="self_team" placeholder="Team ID">
<button name="set_self_team">Set</button>
</form>
</div>
<div class="section">
<form method="POST">
<h3>Aim at Team</h3>
<input name="team_box" placeholder="Target ID">
<button name="aim_team">Aim</button>
</form>
</div>
<div class="section">
<form method="POST">
<h3>Manual Control</h3>
<p>Elevation: <input name="m1" value="{m1_angle:.2f}"> {jog_buttons('m1')}</p>
<p>Azimuth: <input name="m2" value="{m2_angle:.2f}"> {jog_buttons('m2')}</p>
<button>Rotate</button>
</form>
</div>
<div class="section">
<form method="POST">
<h3>Calibration</h3>
<button name="save_zero">Save Zero</button>
<button name="return_zero">Return Zero</button>
</form>
</div>
<div class="section">
<form method="POST">
<h3>Laser</h3>
<button name="laser">Test Laser</button>
</form>
</div>
<div class="section">
<h3>Turret Status</h3>
<p>Elevation: {m1_angle:.2f}°</p>
<p>Azimuth: {m2_angle:.2f}°</p>
</div>
</body>
</html>
""".encode()

# ---------------------- Web Server ----------------------
def serve_web(m1_worker, m2_worker):
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('', 8080))
    s.listen(3)
    logging.info("Web server started on port 8080")

    while True:
        conn, addr = s.accept()
        try:
            msg = b""
            while True:
                part = conn.recv(4096)
                if not part:
                    break
                msg += part
            msg = msg.decode(errors="ignore")
            d = {}
            if msg.startswith("POST"):
                d = parsePOSTdata(msg)
                logging.info(f"POST data from {addr}: {d}")

                if "set_self_team" in d:
                    self_team["id"] = d.get("self_team")
                    logging.info(f"Set self team: {self_team['id']}")

                if "aim_team" in d and d.get("team_box"):
                    try:
                        aim_at_team(m1_worker, m2_worker, d.get("team_box"))
                    except Exception as e:
                        logging.error(f"Aim error: {e}")

                for motor_name, worker in (("m1", m1_worker), ("m2", m2_worker)):
                    if motor_name in d and d[motor_name]:
                        try:
                            delta = float(d[motor_name]) - worker.motor.angle
                            worker.rotate(delta)
                        except Exception as e:
                            logging.error(f"{motor_name} manual rotate error: {e}")

                for k, worker in (("m1_jog", m1_worker), ("m2_jog", m2_worker)):
                    if k in d:
                        try:
                            worker.rotate(float(d[k]))
                        except Exception as e:
                            logging.error(f"{k} jog error: {e}")

                if "save_zero" in d:
                    try:
                        save_zero(m1_worker.motor, m2_worker.motor)
                    except Exception as e:
                        logging.error(f"Save zero error: {e}")

                if "return_zero" in d:
                    try:
                        return_to_zero(m1_worker.motor, m2_worker.motor)
                    except Exception as e:
                        logging.error(f"Return zero error: {e}")

                if "laser" in d:
                    try:
                        test_laser()
                    except Exception as e:
                        logging.error(f"Laser test error: {e}")

            conn.send(b"HTTP/1.1 200 OK\r\nContent-Type:text/html\r\n\r\n")
            conn.sendall(web_page(m1_worker.motor.angle, m2_worker.motor.angle))
        except Exception as e:
            logging.error(f"Web server error for {addr}: {e}")
        finally:
            conn.close()

# ---------------------- Main ----------------------
if __name__ == "__main__":
    shifter = Shifter(23,24,25)
    shifter.last_value = 0
    lock = threading.Lock()

    m1 = Stepper(shifter, lock, 0)
    m2 = Stepper(shifter, lock, 1)
    m1.zero()
    m2.zero()

    m1_worker = MotorWorker(m1)
    m2_worker = MotorWorker(m2)
    m1_worker.start()
    m2_worker.start()

    threading.Thread(target=serve_web, args=(m1_worker, m2_worker), daemon=True).start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logging.info("Shutting down...")
        GPIO.cleanup()
