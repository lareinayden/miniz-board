from common import *
from math import radians,degrees
from Offboard import Offboard
from Joystick import Joystick
from time import time,sleep

class TwocarJoystickDemo(PrintObject):
    def __init__(self):
        self.car1 = Offboard("192.168.0.11",2390)
        self.car2 = Offboard("192.168.0.12",2390)
        return

    def main(self):
        joystick = Joystick()
        try:
            while True:
                self.car1.ready.wait()
                self.car1.ready.clear()
                self.car1.throttle = joystick.throttle
                self.car1.steering = joystick.steering * radians(26.1)
                self.print_info('car 1 command:',self.car1.throttle,self.car1.steering)

                self.car2.ready.wait()
                self.car2.ready.clear()
                self.car2.throttle = -joystick.throttle
                self.car2.steering = -joystick.steering * radians(26.1)
                self.print_info('car 2 command:',self.car2.throttle,self.car2.steering)
        except KeyboardInterrupt:
            self.print_info("waiting to quit")
            joystick.quit()
            self.car1.quit()
            self.car2.quit()

if __name__ == '__main__':
    main = TwocarJoystickDemo() 
    main.main()

