from multiprocessing import Process, Value
from ctypes import c_double
import time
from shifter import Shifter


# 8-step half-step cycle
CYCLE = [
    0b0001,
    0b0011,
    0b0010,
    0b0110,
    0b0100,
    0b1100,
    0b1000,
    0b1001
]

STEPS_PER_REV = 4096
STEP_ANGLE = 360.0 / STEPS_PER_REV      # ~0.088 degrees per step
DELAY = 1200 / 1e6                      # step delay


class Stepper:
    """
    Controls ONE motor (4 bits of shift register).
    Each motor gets a 4-bit slot:
       Motor 1 -> bits 4–7
       Motor 2 -> bits 0–3
    """
    def __init__(self, shifter, bit_offset):
        self.s = shifter
        self.bit_offset = bit_offset          # 0 for motor2, 4 for motor1
        self.pos = 0                           # index in cycle 0–7
        self.angle = Value(c_double, 0.0)      # multiprocessing-safe angle

    def _send_pattern(self, pattern_high, pattern_low):
        """
        Write both motors’ patterns simultaneously.
        Maintaining simultaneous operation requires the main program
        to supply the other motor’s pattern.
        This class ONLY writes the bits for its own motor.
        """
        combined = (pattern_high << 4) | pattern_low
        self.s.shiftByte(combined)

    def _step(self, dir, shared_patterns):
        """
        Execute ONE step in direction dir (±1).
        shared_patterns is a multiprocessing.Value("i")
        that stores the other motor's 4-bit pattern.
        """
        self.pos = (self.pos + dir) % 8
        my_pattern = CYCLE[self.pos]

        if self.bit_offset == 4:    # this motor = high bits
            self._send_pattern(my_pattern, shared_patterns.value)
        else:                        # this motor = low bits
            self._send_pattern(shared_patterns.value, my_pattern)

        time.sleep(DELAY)

        # update angle
        with self.angle.get_lock():
            self.angle.value += dir * STEP_ANGLE

    def rotate(self, steps, shared_patterns):
        """
        Rotate a specific number of steps (positive or negative).
        """
        dir = 1 if steps >= 0 else -1
        for _ in range(abs(steps)):
            self._step(dir, shared_patterns)

    def goAngle(self, target_angle, shared_patterns):
        """
        Move to absolute angle using the SHORTEST PATH.
        """
        # current angle
        with self.angle.get_lock():
            curr = self.angle.value

        # normalize target
        target = target_angle % 360.0
        curr_mod = curr % 360.0

        # compute angle difference using shortest path
        diff = target - curr_mod
        if diff > 180:
            diff -= 360
        if diff < -180:
            diff += 360

        # convert to steps
        steps = int(diff / STEP_ANGLE)
        self.rotate(steps, shared_patterns)

        # force angle correct
        with self.angle.get_lock():
            self.angle.value = target

    def zero(self):
        with self.angle.get_lock():
            self.angle.value = 0.0


def simultaneous(m1_func, m2_func):
    """
    Run two stepper operations in parallel.
    """
    p1 = Process(target=m1_func)
    p2 = Process(target=m2_func)
    p1.start()
    p2.start()
    p1.join()
    p2.join()


# ---------------------------------------------------------
# MAIN EXECUTION
# ---------------------------------------------------------
if __name__ == "__main__":
    s = Shifter(dataPin=23, latchPin=24, clockPin=25)

    # store each motor's 4-bit pattern to allow simultaneous operation
    m1_pat = Value("i", 0)
    m2_pat = Value("i", 0)

    m1 = Stepper(s, bit_offset=4)   # top 4 bits
    m2 = Stepper(s, bit_offset=0)   # bottom 4 bits

    # Example sequence from assignment
    def seq():
        m1.zero()
        m2.zero()

        simultaneous(lambda: m1.goAngle(90, m2_pat),
                     lambda: None)

        simultaneous(lambda: m1.goAngle(-45, m2_pat),
                     lambda: None)

        simultaneous(lambda: None,
                     lambda: m2.goAngle(-90, m1_pat))

        simultaneous(lambda: None,
                     lambda: m2.goAngle(45, m1_pat))

        simultaneous(lambda: m1.goAngle(-135, m2_pat),
                     lambda: None)

        simultaneous(lambda: m1.goAngle(135, m2_pat),
                     lambda: None)

        simultaneous(lambda: m1.goAngle(0, m2_pat),
                     lambda: None)

    seq()
