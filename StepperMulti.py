import time
import multiprocessing
from Shifter import shifter  # your custom module

# Shared array for two steppers (integers)
myArray = multiprocessing.Array('i', 2)

class Stepper:
    seq = [0b0001, 0b0011, 0b0010, 0b0110,
           0b0100, 0b1100, 0b1000, 0b1001]
    delay = 12000  # microseconds
    steps_per_degree = 1024 / 360

    def __init__(self, shifter, lock, index):
        self.s = shifter
        self.lock = lock
        self.index = index
        self.angle = 0
        self.step_state = 0
        self.shifter_bit_start = 4 * index

    def _sgn(self, x):
        return 0 if x == 0 else int(abs(x)/x)

    def _step(self, direction):
        with self.lock:
            self.step_state = (self.step_state + direction) % 8
            # Clear previous 4 bits
            myArray[self.index] &= ~(0b1111 << self.shifter_bit_start)
            # Set new bits
            myArray[self.index] |= (Stepper.seq[self.step_state] << self.shifter_bit_start)

            # Combine all motor bytes
            final = 0
            for val in myArray:
                final |= val

            # Send to shift register
            self.s.shiftByte(final)

            # Update angle
            self.angle = (self.angle + direction / Stepper.steps_per_degree) % 360

        time.sleep(Stepper.delay / 1e6)

    def _rotate(self, delta):
        steps = int(Stepper.steps_per_degree * abs(delta))
        direction = self._sgn(delta)
        for _ in range(steps):
            self._step(direction)

    def rotate(self, delta):
        p = multiprocessing.Process(target=self._rotate, args=(delta,))
        p.start()
        return p

    def zero(self):
        self.angle = 0


if __name__ == '__main__':
    s = shifter(23, 24, 25)
    lock = multiprocessing.Lock()

    m1 = Stepper(s, lock, 0)
    m2 = Stepper(s, lock, 1)

    # Start both motors at once
    p1 = m1.rotate(90)
    p2 = m2.rotate(-90)

    # Wait for both to finish
    p1.join()
    p2.join()

    print("\nBoth motors finished!")


