import socket

def set_state(steer, speed, ip, port=2390):
    '''
    Updates the state of the car.

            Parameters:
                    steer (float): a float from -1.0 to 1.0 where -1.0 is full left and 1.0 is full right
                    speed (int): a float from -1.0 to 1.0 where -1.0 is full reverse and 1.0 is full forward
    '''
    steer = min(steer, 1.0)
    steer = max(steer, -1.0)

    speed = min(speed, 1.0)
    speed = max(speed, -1.0)

    raw_steer = 127.5
    if steer < 0.0:
        raw_steer = (127.5 * (-1.0 - steer)) * -1
    elif steer > 0.0:
        raw_steer = (127.5 * steer) + 127.5
    
    raw_speed = 0.0
    if speed < 0.0:
        raw_speed = speed * 255 * -1
    if speed > 0.0:
        raw_speed = (speed * 255) + 255

    print('{0:03d}'.format(round(raw_speed)) + '{0:03d}'.format(round(raw_steer)))

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(bytes('{0:03d}'.format(round(raw_speed)) + '{0:03d}'.format(round(raw_steer)), "utf-8"), (ip, port))
