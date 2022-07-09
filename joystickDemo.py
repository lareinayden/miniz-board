from common import *
from math import radians,degrees
from Offboard import Offboard
from Joystick import Joystick
from time import time,sleep

class JoystickDemo(PrintObject):
    def __init__(self):
        self.offboard = Offboard("192.168.0.11",2390)
        return

    def main(self):
        joystick = Joystick()
        try:
            while True:
                sleep(0.05)
                throttle = joystick.throttle
                steering = joystick.steering * radians(26.1)
                self.print_info('command:',throttle,steering)
        except KeyboardInterrupt:
            joystick.quit()
            self.offboard.quit()

if __name__ == '__main__':
    main = JoystickDemo() 
    main.main()

