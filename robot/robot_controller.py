import datetime
import socket
import struct

import math
import time

from camera import UsbCamera


class RobotController():

    MSGLEN = 7*2

    def __init__(self,ip,port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((ip, port))

    def sendPosition(self, q):
        controlbytes = 1.0
        msg = struct.pack("<ddddddd",controlbytes,*q)

        totalsent = 0
        while totalsent < RobotController.MSGLEN:
            sent = self.sock.send(msg[totalsent:])
            if sent == 0:
                raise RuntimeError("socket connection broken")
            totalsent = totalsent + sent

    def readPosition(self):
        chunks = []
        bytes_recd = 0
        while bytes_recd < RobotController.MSGLEN:
            chunk = self.sock.recv(min(RobotController.MSGLEN - bytes_recd, 2048))
            if chunk == b'':
                raise RuntimeError("socket connection broken")
            chunks.append(chunk)
            bytes_recd = bytes_recd + len(chunk)
        response = b''.join(chunks)
        return struct.unpack("<ddddddd",response)[1:]


def sinwavegenerator(amplitudes,freq,length_time):
    time_start = time.time()
    while time.time()<time_start+length_time:
        pos = []
        deltatime = time.time()-time_start
        for a,f in zip(amplitudes,freq):
            pos.append(a*math.sin(f*deltatime))
        yield pos

def test_connection():
    rc = RobotController("192.168.1.1", 1000)
    coords = rc.readPosition()
    print(coords)

def test_command():
    rc = RobotController("192.168.1.1", 1000)
    coords = rc.sendPosition([0.0, 0.0, 0.0, 0.0, 0.0])

def experiment_sinus():
    # rc = RobotController("192.168.1.1",1000)
    amplitudes = [5.0, 10.0, 15.0, 10.0, 5.0]
    amplitudes = [math.radians(x) for x in amplitudes]
    freq = [0.1, 0.1, 0.1, 0.1, 0.1]

    file_name = str(datetime.datetime.now()) \
                    .replace(" ", "") \
                    .replace(".", "") \
                    .replace("-", "") \
                    .replace(":", "") + ".txt"

    camera = UsbCamera(0,'WebCam',30)
    camera.recordVideo(5.0)

    with open(file_name, "a+") as log:
        sin_waves = sinwavegenerator(amplitudes, freq, 5.0)
        starttime = time.time()
        while time.time() < starttime + 5.0:
            # rc.sendPosition(next(sin_waves))
            # coords = rc.readPosition()
            coords = next(sin_waves)
            log.write(str(coords))

if __name__ == '__main__':
    experiment_sinus()
