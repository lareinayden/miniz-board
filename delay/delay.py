import socket
from struct import pack, unpack
ip = "192.168.0.100"
port = 2390
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('192.168.0.101',58999))
sent_size = sock.sendto(pack('255s',b'abcd'), (ip, port))

data, addr = sock.recvfrom(1)
print(data)
