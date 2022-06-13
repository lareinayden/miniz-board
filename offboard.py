# suites to communicate with offboard miniz
from common import *
import socket
from struct import pack, unpack
import numpy as np
from time import clock_gettime_ns, CLOCK_REALTIME
from math import degrees,radians
from joystick import Joystick

class OffboardPacket:
    seq_no = 0
    packet_size = 64
    def __init__(self):
        # actual whole packet
        self.seq_no = None
        self.type = None
        self.subtype = None
        self.packet = None
        self.payload = None
        # package encoded ts
        self.ts = None
        return

    def emptyPayload(self):
        self.payload = b''

    # encode all fields into .packet
    def makePacket(self):
        self.seq_no = OffboardPacket.seq_no
        self.ts = int(clock_gettime_ns(CLOCK_REALTIME) / 1000) % 4294967295
        header = pack('IIHH',self.seq_no,self.ts,self.type,self.subtype)
        padding_size = OffboardPacket.packet_size - len(header) - len(self.payload)
        padding = pack('x'*padding_size)
        self.packet = header+self.payload+padding

        OffboardPacket.seq_no += 1

    def parsePacket(self):
        packet = self.packet
        header = packet[:12]
        self.seq_no,self.ts,self.type,self.subtype = unpack('IIHH',header)
        return
        # TODO example: further parsing
        if (self.type == 0):
            # ping packet
            if (self.subtype == 0):
                #ping request
                pass
            elif (self.subtype == 1):
                # ping response
                pass
        if (self.type == 1):
            # command host -> car only
            self.throttle,self.steering = unpack('ff',packet[12:20])

class Offboard(PrintObject):
    def __init__(self):
        self.initSocket()
        #self.initSerial()
        self.printStatus()
        self.joystick = Joystick()

    def initSocket(self):
        self.local_ip = "192.168.0.101"
        self.local_port = 58999
        self.car_ip = "192.168.0.100"
        self.car_port = 2390
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((self.local_ip, self.local_port))
        self.sock = sock

    def printStatus(self):
        return

    def main(self):
        self.delay_vec = []
        try:
            while True:
                self.loop()
        except KeyboardInterrupt:
            self.joystick.quit()
        print_ok("average delay (us) = ", np.mean(self.delay_vec))

    def loop(self):
        # read joystick
        throttle = self.joystick.throttle
        steering = self.joystick.steering * radians(26.1)
        print(throttle,steering)
        # prepare packet
        #packet = self.preparePingPacket()
        #packet = self.prepareCommandPacket(throttle,steering)
        packet = self.prepareCommandPacket(0.2,steering)
        # send
        self.sendPacket(packet)
        #self.print_ok("sent packet no.%d"%count)
        # wait for response
        response = self.getResponse()
        #self.print_ok("got response", response)
        # parse packet
        response_packet = self.parseResponse(response)
        # log delay
        delay = self.last_response_ts - self.last_sent_ts 
        self.print_ok("delay (us) %d"%(delay))
        self.delay_vec.append(delay)

    def sendPacket(self,packet):
        sent_size = self.sock.sendto(packet.packet, (self.car_ip, self.car_port))
        self.last_sent_ts = packet.ts

    def getResponse(self):
        data, addr = self.sock.recvfrom(OffboardPacket.packet_size)
        return data

    def parseResponse(self,data):
        packet = OffboardPacket()
        packet.packet = data
        packet.parsePacket()
        self.last_response_ts = int(clock_gettime_ns(CLOCK_REALTIME) / 1000) % 4294967295
        return packet

    def preparePingPacket(self):
        packet = OffboardPacket()
        packet.type = 0
        packet.subtype = 0
        packet.emptyPayload()
        packet.makePacket()
        return packet

    def prepareCommandPacket(self,throttle=0.0,steering=radians(10)):
        packet = OffboardPacket()
        packet.type = 1
        packet.subtype = 1886
        packet.payload = pack('ff',throttle,steering)
        packet.makePacket()
        return packet
        
if __name__ == '__main__':
    main = Offboard()
    main.main()



