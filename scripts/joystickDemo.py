from common import *
from math import radians,degrees
from Offboard import Offboard
from Joystick import Joystick
from time import time,sleep

class JoystickDemo(PrintObject):
    def __init__(self):
        print("INIT...")
        self.joystick = Joystick()
        self.car = Offboard("192.168.10.12",28840)
        print("DID INIT")
        return

    def main(self):
        print("JS")
        print("DID JS")
        try:
            while True:
                print("DEMO TICK")
                self.joystick.updateOnce()
                self.car.ready.wait()
                self.car.ready.clear()
                self.car.throttle = self.joystick.throttle * 3
                self.car.steering = self.joystick.steering * radians(26.1)
                print('command:',self.car.throttle,self.car.steering)
        except KeyboardInterrupt:
            self.print_info("waiting to quit")
            self.joystick.quit()
            self.car.quit()
        self.car.final()

if __name__ == '__main__':
    main = JoystickDemo() 
    main.main()

