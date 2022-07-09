# send sinusoidal steering command
from math import pi,radians,degrees,sin
from common import *
from Offboard import Offboard
from time import time,sleep


class SinSteeringTest(PrintObject):
    def __init__(self,T):
        self.car = Offboard("192.168.0.11",2390)
        self.T = T
        return

    def main(self):
        try:
            while True:
                self.car.ready.wait()
                self.car.ready.clear()
                self.car.throttle = 0.0
                self.car.steering = sin(2*pi/self.T*time()) * radians(26.1)
                self.print_info('command:',self.car.throttle,self.car.steering)
        except KeyboardInterrupt:
            self.print_info("waiting to quit")
            self.car.quit()
        self.car.final()

if __name__ == '__main__':
    T = 1.0
    main = SinSteeringTest(T) 
    main.main()

