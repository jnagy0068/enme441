import time
import socket
import threading
import json
import math
import os
import logging
from urllib.parse import parse_qs

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
    delay = 0.001
    steps_per_degree = 4 * 1024 / 360

    def __init__(self, shifter, lock, index):
        self.s = shifter
        self.lock = lock
        self.index = index
        self.angle = 0.0
        self.step_state = 0
        self.bit_start = 4 * index

    def _sgn(self, x):
        return 0 if x == 0 else int(abs(x)/x)

    def _step(self, direction):
        with self.lock:
            self.step_state = (self.step_state + direction) % 8
            bits = Stepper.seq[self.step_state] << self.bit_start
            final = self.s.last_value & ~(0b1111 << self.bit_start)
            final |= bits
            self.s.shiftByte(final)
            self.s.last_value = final
        self.angle = (self.angle + direction/Stepper.steps_per_degree) % 360
        logging.debug(f"Motor {self.index} step: angle={self.angle:.2f}")
        time.sleep(Stepper.delay)

    def _rotate(self, delta):
        if abs(delta) < 0.01:  # ignore tiny moves
            return
        direction = self._sgn(delta)
        steps = int(abs(delta)*Stepper.steps_per_degree)
        for _ in range(steps):
            self._step(direction)

    def rotate(self, delta):
        if abs(delta) < 0.01:
            return
        t = threading.Thread(target=self._rotate, args=(delta,), daemon=True)
        t.start()
        return t

    def goAngle(self, target):
        delta = (target - self.angle + 540) % 360 - 180
        return self.rotate(delta)

    def zero(self):
        self.angle = 0.0

# ---------------------- Laser ----------------------
def test_laser():
    logging.info("Laser test triggered")
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
    m1.goAngle(zero_angles["m1"] + calibration_offsets["m1"])
    m2.goAngle(zero_angles["m2"] + calibration_offsets["m2"])
    logging.info("Returning to zero.")

# ---------------------- Aim at Team ----------------------
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
    r_tgt = positions["turrets"][target_team]["r"]
    th_tgt = positions["turrets"][target_team]["theta"]

    x_self = r_self*math.cos(th_self)
    y_self = r_self*math.sin(th_self)
    x_tgt = r_tgt*math.cos(th_tgt)
    y_tgt = r_tgt*math.sin(th_tgt)
    dx = x_tgt - x_self
    dy = y_tgt - y_self
    dz = turret_height_other - turret_height_self

    az_deg = math.degrees(math.atan2(dy, dx)) + calibration_offsets["m2"]
    el_deg = -math.degrees(math.atan2(dz, math.hypot(dx, dy))) + calibration_offsets["m1"]

    m1.goAngle(el_deg)
    m2.goAngle(az_deg)
    logging.info(f"Aiming at team {target_team}: az={az_deg:.2f}, el={el_deg:.2f}")

# ---------------------- POST Parsing ----------------------
def parsePOSTdata(data):
    idx = data.find("\r\n\r\n")
    if idx == -1:
        return {}
    parsed = parse_qs(data[idx+4:], keep_blank_values=True)
    return {k:v[0] for k,v in parsed.items()}

# ---------------------- Web Page ----------------------
def web_page_safe(m1, m2):
    try: m1_angle = getattr(m1,"angle",0.0)
    except: m1_angle = 0.0
    try: m2_angle = getattr(m2,"angle",0.0)
    except: m2_angle = 0.0

    def jog_buttons(name):
        vals = [-90,-45,-15,-5,-1,-0.5,0.5,1,5,15,45,90]
        return " ".join(f'<button name="{name}_jog" value="{v}">{v:+}°</button>' for v in vals)

    self_id = self_team.get("id","N/A")
    target_id = current_target_team.get("id","N/A")

    return f"""
<html><body style="font-family:Arial;text-align:center;">
<h2>Laser Turret</h2>
<p>Self: {self_id} | Target: {target_id}</p>
<form method="POST">
<h3>Set Team</h3>
<input name="self_team"><button name="set_self_team">Set</button>
<h3>Aim</h3>
<input name="team_box"><button name="aim_team">Aim</button>
<h3>Manual</h3>
Az <input name="m2" value="{m2_angle:.2f}">{jog_buttons('m2')}<br>
El <input name="m1" value="{m1_angle:.2f}">{jog_buttons('m1')}<br>
<h3>Calibration</h3>
<button name="save_zero">Save Zero</button>
<button name="return_zero">Return Zero</button>
<h3>Laser</h3>
<button name="laser">Test</button>
</form>
</body></html>
""".encode()

# ---------------------- Web Server ----------------------
def serve_web(m1, m2):
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
    s.bind(('',8080))
    s.listen(3)
    logging.info("Web server started on port 8080")

    while True:
        conn,_ = s.accept()
        try:
            msg = conn.recv(4096).decode(errors="ignore")
            if msg.startswith("POST"):
                d = parsePOSTdata(msg)

                if "set_self_team" in d:
                    self_team["id"] = d.get("self_team")

                if "aim_team" in d and d.get("team_box"):
                    aim_at_team(m1,m2,d.get("team_box"))

                # Manual control
                for motor_name, motor in (("m1",m1),("m2",m2)):
                    if motor_name in d and d[motor_name]:
                        try:
                            target = float(d[motor_name])
                            if abs(target - motor.angle) > 0.01:
                                motor.goAngle(target)
                        except: pass

                # Jog buttons
                for k,motor in (("m1_jog",m1),("m2_jog",m2)):
                    if k in d:
                        try:
                            motor.rotate(float(d[k]))
                        except: pass

                # Calibration
                if "save_zero" in d:
                    save_zero(m1,m2)
                if "return_zero" in d:
                    return_to_zero(m1,m2)

                # Laser
                if "laser" in d:
                    test_laser()

            conn.send(b"HTTP/1.1 200 OK\r\nContent-Type:text/html\r\n\r\n")
            conn.sendall(web_page_safe(m1,m2))
        except Exception as e:
            logging.error(f"Web server error: {e}")
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

    threading.Thread(target=serve_web, args=(m1,m2), daemon=True).start()

    try:
        while True: time.sleep(1)
    except KeyboardInterrupt:
        logging.info("Shutting down...")
        GPIO.cleanup()
