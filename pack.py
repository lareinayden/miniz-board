from struct import pack, unpack
from time import clock_gettime_ns, CLOCK_REALTIME
packet_size = 32
seq_no = 998
ts = int(clock_gettime_ns(CLOCK_REALTIME) / 1000) % 4294967295
p_type = 1
p_subtype = 2
throttle = 3.3
steering = 2

# encoding packet
print("sending")
print(seq_no,ts,p_type,p_subtype)
header = pack('IIHH',seq_no,ts,p_type,p_subtype)
payload = pack('ff',throttle,steering)
padding_size = packet_size - len(header) - len(payload)
padding = pack('x'*padding_size)
packet = header+payload+padding
print(header,len(header))
print(payload,len(payload))
print(padding,len(padding))
print(packet,len(packet))

# decoding packet
print("recving")
header = packet[:12]
seq_no,ts,p_type,p_subtype = unpack('IIHH',header)
throttle,steering = unpack('ff',packet[12:20])
print(seq_no,ts,p_type,p_subtype,throttle,steering)


