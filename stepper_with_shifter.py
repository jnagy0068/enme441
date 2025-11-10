from multiprocessing import Process, Value, Lock
from ctypes import c_double
import time
from shifter import Shifter


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
    def __init__(self, shifter, bit_offset, global_output, lock):
        self.s = shifter
        self.bit_offset = bit_offset              # 4 for motor1, 0 for motor2
        self.pos = 0
        self.angle = Value(c_double, 0.0)
        self.global_output = global_output        # shared 8-bit register
        self.lock = lock                          # motor-specific lock

    def _step(self, direction):
        self.pos = (self.pos + direction) % 8
        my_pattern = CYCLE[self.pos]

        with self.lock:   # THIS MOTOR ONLY
            # read current 8-bit state
            current = self.global_output.value

            if self.bit_offset == 4:   # motor 1 (upper 4 bits)
                new_output = (my_pattern << 4) | (current & 0x0F)
            else:                      # motor 2 (lower 4 bits)
                new_output = (current & 0xF0) | my_pattern

            # write updated global byte
            self.global_output.value = new_output

            # shift out
            self.s.shiftByte(new_output)

        time.sleep(DELAY)

        with self.angle.get_lock():
            self.angle.value += direction * STEP_ANGLE

    def rotate(self, steps):
        direction = 1 if steps >= 0 else -1
        for _ in range(abs(steps)):
            self._step(direction)

    def goAngle(self, target):
        with self.angle.get_lock():
            curr = self.angle.value % 360.0

        target = target % 360.0
        diff = target - curr
        if diff > 180:
            diff -= 360
        if diff < -180:
            diff += 360

        steps = int(diff / STEP_ANGLE)
        self.rotate(steps)

        with self.angle.get_lock():
            self.angle.value = target

    def zero(self):
        with self.angle.get_lock():
            self.angle.value = 0.0


def simultaneous(f1, f2):
    p1 = Process(target=f1)
    p2 = Process(target=f2)
    p1.start()
    p2.start()
    p1.join()
    p2.join()


if __name__ == "__main__":
    s = Shifter(dataPin=23, latchPin=24, clockPin=25)

    global_output = Value("i", 0)      # entire 74HC595 byte
    m1_lock = Lock()
    m2_lock = Lock()

    m1 = Stepper(s, bit_offset=4, global_output=global_output, lock=m1_lock)
    m2 = Stepper(s, bit_offset=0, global_output=global_output, lock=m2_lock)

    def seq():
        m1.zero()
        m2.zero()

        simultaneous(
            lambda: m1.goAngle(90),
            lambda: m2.goAngle(-90)
        )

        simultaneous(
            lambda: m1.goAngle(-45),
            lambda: m2.goAngle(45)
        )

        simultaneous(
            lambda: m1.goAngle(-135),
            lambda: m2.goAngle(0)
        )

        simultaneous(
            lambda: m1.goAngle(0),
            lambda: m2.goAngle(0)
        )

    seq()
