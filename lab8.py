import time
import sys
import multiprocessing
import RPi.GPIO as GPIO
from shifter import Shifter

shared_states = multiprocessing.Array('i', 2)

class Stepper:
    seq = [0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001]

    base_delay = 1000 #effects motor turn speed, too low can cause stalling
    steps_per_degree = 1024 / 360

    def __init__(self, shifter, lock, index):
        self.s = shifter
        self.lock = lock
        self.index = index 
        self.angle = multiprocessing.Value('d', 0.0)
        self.step_state = 0
        self.bit_offset = 4 * index

    def _sgn(self, x):
        return 0 if x == 0 else int(abs(x) / x)

    def _step(self, direction):
        self.step_state = (self.step_state + direction) % 8
        pattern = Stepper.seq[self.step_state] << self.bit_offset

        with self.lock:
            mask = 0b1111 << self.bit_offset
            shared_states[self.index] = (shared_states[self.index] & ~mask) | pattern

            combined = 0
            for v in shared_states:
                combined |= v

            self.s.shiftByte(combined)

        with self.angle.get_lock():
            self.angle.value = (self.angle.value + direction / Stepper.steps_per_degree) % 360

        time.sleep(Stepper.base_delay / 1e6)

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
        with self.angle.get_lock():
            current = self.angle.value

        delta = (target - current + 540) % 360 - 180
        return self.rotate(delta)

    def zero(self):
        with self.angle.get_lock():
            self.angle.value = 0.0


if __name__ == '__main__':
    try:
        s = Shifter(23, 24, 25)
        lock1 = multiprocessing.Lock()
        lock2 = multiprocessing.Lock()

        m1 = Stepper(s, lock1, 0)
        m2 = Stepper(s, lock2, 1)

        m1.zero()
        m2.zero()

        p1 = m1.goAngle(90)
        p2 = m2.goAngle(-90)
        p1.join()
        p2.join()

        p1 = m1.goAngle(-45)
        p1.join()

        p1 = m1.goAngle(135)
        p2 = m2.goAngle(45)
        p1.join()
        p2.join()

        p1 = m1.goAngle(0)
        p1.join()

    except KeyboardInterrupt:
    finally:
        GPIO.cleanup()
        sys.exit(0)
