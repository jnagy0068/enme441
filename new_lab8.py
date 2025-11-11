#Partners glen ahern and lucas rose

import time
import multiprocessing
from shifter import Shifter  

class Stepper:
   
    numsteppers = 0
    shifteroutputs = 0
    seq = [0b0001,0b0011,0b0010,0b0110,0b0100,0b1100,0b1000,0b1001]
    delay = 1000     
    stepsperdegree = 4096/360

    def __init__(self, shifter, lock):
        self.s = shifter
        self.angle = multiprocessing.Value('d', 0.0) 
        self.stepstate = 0
        self.shifterbitstart = 4*Stepper.numsteppers
        self.lock = lock
        Stepper.numsteppers += 1

    def __sgn(self, x):
        return 0 if x == 0 else int(abs(x)/x)

    def __step(self, dir):
        self.stepstate = (self.stepstate + dir) % 8

    
        Stepper.shifteroutputs &= ~(0b1111 << self.shifterbitstart)
        Stepper.shifteroutputs |= (Stepper.seq[self.stepstate] << self.shifterbitstart)
        self.s.shiftByte(Stepper.shifteroutputs)


        with self.angle.get_lock():
            self.angle.value += dir / Stepper.stepsperdegree
            self.angle.value %= 360

    def __rotate(self, delta):
        numSteps = int(Stepper.stepsperdegree * abs(delta))
        dir = self.__sgn(delta)
        for _ in range(numSteps):
            self.__step(dir)
            time.sleep(Stepper.delay / 1e6)

    def rotate(self, delta):

        time.sleep(0.1)
        p = multiprocessing.Process(target=self.__rotate, args=(delta,))
        p.start()
        return p 

    def goAngle(self, target_angle):

        with self.angle.get_lock():
            current_angle = self.angle.value

        delta = (target_angle - current_angle) % 360
        if delta > 180:
            delta -= 360

        return self.rotate(delta)

    def zero(self):
        with self.angle.get_lock():
            self.angle.value = 0.0

if __name__ == '__main__':
    s = Shifter(dataPin=23, latchPin=24, clockPin=25)
    lock = multiprocessing.Lock()

    m1 = Stepper(s, lock)
    m2 = Stepper(s, lock)
    m1.zero()
    m2.zero()

    sequence = [
        (90, -45),
        (-90, 45),
        (-135, 135),
        (0, 0)
    ]

    for angle1, angle2 in sequence:

        p1 = m1.goAngle(angle1)
        p2 = m2.goAngle(angle2)

        p1.join()
        p2.join()
        print(f"Motors reached angles: M1={angle1}, M2={angle2}")

#Partners glen and lucas
