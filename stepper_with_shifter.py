from multiprocessing import Process, Value
from ctypes import c_double
import time
from shifter import Shifter


# 8-step half-step sequence
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
STEP_ANGLE = 360.0 / STEPS_PER_REV
DELAY = 1200 / 1e6


class Stepper:
    """
    Controls ONE motor on a 74HC595 shift register.
    bit_offset = 4  → high nybble (motor 1)
    bit_offset = 0  → low nybble  (motor 2)
    """

    def __init__(self, shifter, bit_offset):
        self.s = shifter
        self.bit_offset = bit_offset
        self.pos = 0                              # index in CYCLE (0–7)
        self.angle = Value(c_double, 0.0)         # multiprocessing-safe

    def _step(self, direction, my_shared, other_shared):
        """
        Perform ONE step in 'direction'.
        Update shared pattern BEFORE output.
        Read other motor's pattern.
        Output one combined 8-bit pattern.
        """
        # update cycle index
        self.pos = (self.pos + direction) % 8
        my_pattern = CYCLE[self.pos]

        # write my pattern to shared memory
        my_shared.value = my_pattern

        # read other motor's pattern
        other_pattern = other_shared.value

        # combine into 8-bit output
        if self.bit_offset == 4:    # I occupy high bits
            combined = (my_pattern << 4) | other_pattern
        else:                       # I occupy low bits
            combined = (other_pattern << 4) | my_pattern

        # send to shift register
        self.s.shiftByte(combined)
        time.sleep(DELAY)

        # update angle
        with self.angle.get_lock():
            self.angle.value += direction * STEP_ANGLE

    def rotate(self, steps, my_shared, other_shared):
        direction = 1 if steps >= 0 else -1
        for _ in range(abs(steps)):
            self._step(direction, my_shared, other_shared)

    def goAngle(self, target_angle, my_shared, other_shared):
        """
        Move to absolute angle using shortest path.
        """
        with self.angle.get_lock():
            curr = self.angle.value % 360.0

        target = target_angle % 360.0

        diff = target - curr
        if diff > 180:
            diff -= 360
        if diff < -180:
            diff += 360

        steps = int(diff / STEP_ANGLE)
        self.rotate(steps, my_shared, other_shared)

        # force angle to be exact target
        with self.angle.get_lock():
            self.angle.value = target

    def zero(self):
        with self.angle.get_lock():
            self.angle.value = 0.0


def simultaneous(func1, func2):
    """
    Run two functions in parallel.
    """
    p1 = Process(target=func1)
    p2 = Process(target=func2)
    p1.start()
    p2.start()
    p1.join()
    p2.join()


# ---------------------------------------------------------
# MAIN PROGRAM
# ---------------------------------------------------------
if __name__ == "__main__":
    s = Shifter(dataPin=23, latchPin=24, clockPin=25)

    # shared 4-bit patterns for both motors
    m1_pat = Value("i", 0)   # motor 1's pattern
    m2_pat = Value("i", 0)   # motor 2's pattern

    # high bits = motor1, low bits = motor2
    m1 = Stepper(s, bit_offset=4)
    m2 = Stepper(s, bit_offset=0)

    # assignment command sequence
    def sequence():
        m1.zero()
        m2.zero()

        simultaneous(
            lambda: m1.goAngle(90, m1_pat, m2_pat),
            lambda: m2.goAngle(0, m2_pat, m1_pat)
        )

        simultaneous(
            lambda: m1.goAngle(-45, m1_pat, m2_pat),
            lambda: m2.goAngle(0, m2_pat, m1_pat)
        )

        simultaneous(
            lambda: m1.goAngle(0, m1_pat, m2_pat),
            lambda: m2.goAngle(-90, m2_pat, m1_pat)
        )

        simultaneous(
            lambda: m1.goAngle(0, m1_pat, m2_pat),
            lambda: m2.goAngle(45, m2_pat, m1_pat)
        )

        simultaneous(
            lambda: m1.goAngle(-135, m1_pat, m2_pat),
            lambda: m2.goAngle(0, m2_pat, m1_pat)
        )

        simultaneous(
            lambda: m1.goAngle(135, m1_pat, m2_pat),
            lambda: m2.goAngle(0, m2_pat, m1_pat)
        )

        simultaneous(
            lambda: m1.goAngle(0, m1_pat, m2_pat),
            lambda: m2.goAngle(0, m2_pat, m1_pat)
        )

    sequence()
