from common import *
from math import radians,degrees
from Offboard import Offboard
from Joystick import Joystick
from time import time,sleep

class JoystickDemo(PrintObject):
    def __init__(self):
        self.car = Offboard("192.168.0.11",2390)
        return

    def main(self):
        joystick = Joystick()
        try:
            while True:
                self.car.ready.wait()
                self.car.ready.clear()
                self.car.throttle = joystick.throttle
                self.car.steering = joystick.steering * radians(26.1)
                self.print_info('command:',self.car.throttle,self.car.steering)
        except KeyboardInterrupt:
            self.print_info("waiting to quit")
            joystick.quit()
            self.car.quit()
        self.car.final()

if __name__ == '__main__':
    main = JoystickDemo() 
    main.main()

