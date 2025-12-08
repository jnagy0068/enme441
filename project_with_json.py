import time
import multiprocessing
import socket
import threading
import json
from shifter import Shifter
import RPi.GPIO as GPIO
import os
from urllib.parse import parse_qs

GPIO.setmode(GPIO.BCM)
laser = 22
GPIO.setup(laser, GPIO.OUT)
GPIO.output(laser, GPIO.LOW)

myArray = multiprocessing.Array('i', 2)

positions = {}
calibration = {"az_offset": 0.0, "el_offset": 0.0}
self_team = {"id": None}


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


def test_laser():
    GPIO.output(laser, GPIO.HIGH)
    time.sleep(3)
    GPIO.output(laser, GPIO.LOW)


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


def aim_at_team(m1, m2, target_team):
    if self_team["id"] is None:
        print("ERROR: Self team number not set.")
        return

    if target_team not in positions.get("turrets", {}):
        print("Team not found in JSON:", target_team)
        return

    st = self_team["id"]
    if st not in positions["turrets"]:
        print("ERROR: This turret's team number not in JSON:", st)
        return

    th_self = positions["turrets"][st]["theta"]
    th_tgt = positions["turrets"][target_team]["theta"]

    rel_theta = (th_tgt - th_self)
    az = rel_theta * 180.0 / 3.1415926535
    el = 0

    az += calibration["az_offset"]
    el += calibration["el_offset"]

    print(f"Aiming at team {target_team}: az={az:.2f}, el={el:.2f}")

    p1 = m1.goAngle(el)
    p2 = m2.goAngle(az)
    p1.join()
    p2.join()


def parsePOSTdata(data):
    idx = data.find("\r\n\r\n")
    if idx == -1:
        return {}
    post = data[idx + 4:]
    parsed = parse_qs(post, keep_blank_values=True)
    return {k: v[0] for k, v in parsed.items()}


def web_page(m1_angle, m2_angle):
    html = f"""
    <html>
    <head><title>Laser Turret</title></head>

    <body style="font-family: Arial; text-align:center; margin-top:40px;">

        <h2>Laser Turret Control</h2>

        <form action="/" method="POST">

            <h3>This Turret's Team Number</h3>
            <input type="text" name="self_team" placeholder="Your team number"><br><br>
            <input type="submit" name="set_self_team" value="Set This Turret's Team"><br><br>

            <h3>Aim at Another Team</h3>
            <input type="text" name="team_box" placeholder="Target team #"><br><br>
            <input type="submit" name="aim_team" value="Aim at Team"><br><br>

            <h3>Manual Motor Control</h3>

            <h4>Elevation (Motor 1)</h4>
            <input type="text" name="m1" value="{m1_angle}"><br><br>

            <button name="m1_jog" value="-90">-90°</button>
            <button name="m1_jog" value="-45">-45°</button>
            <button name="m1_jog" value="-15">-15°</button>
            <button name="m1_jog" value="-5">-5°</button>
            <button name="m1_jog" value="-1">-1°</button>
            <br><br>
            <button name="m1_jog" value="1">+1°</button>
            <button name="m1_jog" value="5">+5°</button>
            <button name="m1_jog" value="15">+15°</button>
            <button name="m1_jog" value="45">+45°</button>
            <button name="m1_jog" value="90">+90°</button>

            <br><br><br>

            <h4>Azimuth (Motor 2)</h4>
            <input type="text" name="m2" value="{m2_angle}"><br><br>

            <button name="m2_jog" value="-90">-90°</button>
            <button name="m2_jog" value="-45">-45°</button>
            <button name="m2_jog" value="-15">-15°</button>
            <button name="m2_jog" value="-5">-5°</button>
            <button name="m2_jog" value="-1">-1°</button>
            <br><br>
            <button name="m2_jog" value="1">+1°</button>
            <button name="m2_jog" value="5">+5°</button>
            <button name="m2_jog" value="15">+15°</button>
            <button name="m2_jog" value="45">+45°</button>
            <button name="m2_jog" value="90">+90°</button>

            <br><br><br>

            <input type="submit" value="Rotate Motors"><br><br>

            <h3>Calibration</h3>
            <input type="submit" name="return_zero" value="Return to Zero"><br><br>
            <input type="submit" name="save_zero" value="Save Current Position as Zero"><br><br>

            <h3>Laser</h3>
            <input type="submit" name="laser" value="Test Laser (3s)"><br><br>

        </form>
    </body>
    </html>
    """
    return bytes(html, "utf-8")


def serve_web(m1, m2):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("", 8080))
    s.listen(3)
    print("running at IP:8080")

    while True:
        conn, addr = s.accept()
        try:
            msg = conn.recv(4096).decode(errors="ignore")
        except:
            conn.close()
            continue

        if msg.startswith("POST"):
            data = parsePOSTdata(msg)

            if "set_self_team" in data:
                if data.get("self_team", "").strip() != "":
                    self_team["id"] = data["self_team"].strip()
                    print("This turret's team set to:", self_team["id"])

            if "aim_team" in data:
                t = data.get("team_box", "").strip()
                if t != "":
                    aim_at_team(m1, m2, t)

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

            if "m1" in data and data["m1"].strip() != "":
                try:
                    el = float(data["m1"])
                    p = m1.goAngle(el)
                    p.join()
                except:
                    pass

            if "m2" in data and data["m2"].strip() != "":
                try:
                    az = float(data["m2"])
                    p = m2.goAngle(az)
                    p.join()
                except:
                    pass

            if "return_zero" in data:
                return_to_zero(m1, m2)

            if "save_zero" in data:
                save_zero(m1, m2)

            if "laser" in data:
                test_laser()

        try:
            response = web_page(m1.angle, m2.angle)
            conn.send(b"HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n\r\n")
            conn.sendall(response)
        except:
            pass

        conn.close()


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
