import time
import sys
import multiprocessing
import RPi.GPIO as GPIO
from shifter import Shifter  # your custom module

# Shared array for both steppers (each int holds 4 bits of output)
shared_states = multiprocessing.Array('i', 2)

class Stepper:
    # Half-step pattern for 28BYJ-48
    seq = [0b0001, 0b0011, 0b0010, 0b0110,
           0b0100, 0b1100, 0b1000, 0b1001]

    base_delay = 1500  # microseconds between steps (faster = lower)
    steps_per_degree = 1024 / 360  # 28BYJ-48 gear ratio

    def __init__(self, shifter, lock, index):
        self.s = shifter
        self.lock = lock
        self.index = index       # 0 or 1 for motor position
        self.angle = multiprocessing.Value('d', 0.0)  # shared angle (double)
        self.step_state = 0
        self.bit_offset = 4 * index

    def _sgn(self, x):
        return 0 if x == 0 else int(abs(x) / x)

    def _step(self, direction):
        """Perform one step in the given direction."""
        self.step_state = (self.step_state + direction) % 8
        pattern = Stepper.seq[self.step_state] << self.bit_offset

        with self.lock:
            mask = 0b1111 << self.bit_offset
            shared_states[self.index] = (shared_states[self.index] & ~mask) | pattern

            # Combine both motors’ states
            combined = 0
            for v in shared_states:
                combined |= v

            self.s.shiftByte(combined)

        # Update angle safely (within shared memory)
        with self.angle.get_lock():
            self.angle.value = (self.angle.value + direction / Stepper.steps_per_degree) % 360

        time.sleep(Stepper.base_delay / 1e6)

    def _rotate(self, delta):
        """Rotate motor by a relative angle delta (degrees)."""
        direction = self._sgn(delta)
        steps = int(abs(delta) * Stepper.steps_per_degree)
        for _ in range(steps):
            self._step(direction)

    def rotate(self, delta):
        """Launch process to rotate by relative angle delta."""
        p = multiprocessing.Process(target=self._rotate, args=(delta,))
        p.start()
        return p

    def goAngle(self, target):
        """Rotate motor to an absolute angle (shortest path)."""
        with self.angle.get_lock():
            current = self.angle.value

        # Compute shortest angular difference
        delta = (target - current + 540) % 360 - 180  # range (-180, 180]
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

        print("Zeroing motors...")
        m1.zero()
        m2.zero()

        # Example sequence from Lab 8
        cmds = [
            (m1, 90), (m1, -45),
            (m2, -90), (m2, 45),
            (m1, -135), (m1, 135),
            (m1, 0)
        ]

        # Run each command; motors can overlap
        for motor, angle in cmds:
            p = motor.goAngle(angle)
            p.join()
            print(f"Motor {motor.index} → {angle}°")

        print("✅ All commands complete!")

    except KeyboardInterrupt:
        print("\n❌ Interrupted by user.")
    finally:
        GPIO.cleanup()
        sys.exit(0)
