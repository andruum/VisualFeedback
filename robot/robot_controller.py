import datetime
import operator
import socket
import struct

import math
import time

from camera import UsbCamera


class RobotController():

    MSGLEN = 6*8
    ORIGINS = [2.9496, 1.1345, -2.5482, 1.7890, 2.9234];

    def __init__(self,ip,port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((ip, port))

    def sendPosition(self, q):
        controlbytes = 1.0
        msg = struct.pack("<dddddd",controlbytes,*q)

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
        positions = struct.unpack("<dddddd",response)[1:]
        return list(map(operator.sub,positions,RobotController.ORIGINS))


def sinwavegenerator(amplitudes,freq,length_time):
    time_start = time.time()
    while time.time()<time_start+length_time:
        pos = []
        deltatime = time.time()-time_start
        for a,f in zip(amplitudes,freq):
            pos.append(a*math.sin(f*deltatime))
        yield pos

def sinwavegenerator_single(amplitude,freq,length_time):
    time_start = time.time()
    while time.time()<time_start+length_time:
        deltatime = time.time()-time_start
        yield amplitude*math.sin(freq*deltatime)

def test_connection():
    rc = RobotController("192.168.0.183", 51449)
    coords = rc.readPosition()
    print(coords)

def test_command():
    rc = RobotController("192.168.0.183", 51449)
    rc.sendPosition([0.0, 0.0, 0.0, 0.5, 0.0])
    start_time = time.time()
    while start_time+1.0 > time.time():
        rc.sendPosition([0.0, 0.0, 0.0, 0.5, 0.0])
        coords = rc.readPosition()
        print(coords)


def experiment_sinsingle():
    rc = RobotController("192.168.0.183", 51449)

    time_length = 10.0
    sin_wave = sinwavegenerator_single(0.5, 0.5 , time_length)
    start_time = time.time()
    while start_time + time_length > time.time():
        rc.sendPosition([0.0, 0.0, 0.0, next(sin_wave), 0.0])
        coords = rc.readPosition()
        print(coords)

def experiment_sinus():
    rc = RobotController("192.168.0.183", 51449)
    amplitudes = [0.5, 0.5, 0.5, 0.5, 0.5]
    freq = [0.1, 0.1, 0.1, 0.1, 0.1]
    time_length = 10.0

    file_name = str(datetime.datetime.now()) \
                    .replace(" ", "") \
                    .replace(".", "") \
                    .replace("-", "") \
                    .replace(":", "") + ".txt"

    camera = UsbCamera("http://192.168.137.84:8080/video", 'TecnoInf640', 15)
    camera.recordVideo(60.0)

    with open(file_name, "a+") as log:
        sin_waves = sinwavegenerator(amplitudes, freq, time_length)
        starttime = time.time()
        while time.time() < starttime + time_length:
            rc.sendPosition(next(sin_waves))
            coords = rc.readPosition()
            log.write(str(coords))
            log.write("\n")

def writeExampleLog():
    file_name = str(datetime.datetime.now()) \
                    .replace(" ", "") \
                    .replace(".", "") \
                    .replace("-", "") \
                    .replace(":", "") + ".txt"

    with open(file_name, "a+") as log:
        time_length = 25.0
        starttime = time.time()
        amplitudes = [0.5, 0.5, 0.5, 0.5, 0.5]
        freq = [0.5, 0.5, 0.5, 0.5, 0.5]
        sin_waves = sinwavegenerator(amplitudes, freq, time_length)
        log_step = 0.01
        last_write = 0
        while time.time() < starttime + time_length:
            if time.time() > last_write+log_step:
                last_write = time.time()
                coords_with_time = next(sin_waves)
                coords_with_time.append(last_write)
                log.write(str(coords_with_time))
                log.write("\n")


if __name__ == '__main__':
    writeExampleLog()
