from math import cos,sin,radians
import numpy as np


def fromRPYtoRmat(roll,pitch,yaw):
    roll = radians(roll)
    pitch = radians(pitch)
    yaw = radians(yaw)
    Rz=np.asarray([[cos(roll), -sin(roll), 0],
        [sin(roll), cos(roll),0],
        [0,0,1]]);
    Ry = np.asarray([[cos(pitch), 0, sin(pitch)],
          [0, 1, 0],
          [-sin(pitch), 0, cos(pitch)]]);
    Rx = np.asarray([[1, 0, 0],
          [0, cos(yaw), -sin(yaw)],
          [0,sin(yaw),cos(yaw)]]);

    return np.matmul(np.matmul(Rz,Ry),Rx)


def fromZYZtoRmat(Z1,Y,Z2):
    Z1 = radians(Z1)
    Y = radians(Y)
    Z2 = radians(Z2)
    Rz1 = np.asarray([[cos(Z1), -sin(Z1), 0],
                     [sin(Z1), cos(Z1), 0],
                     [0, 0, 1]]);
    Ry = np.asarray([[cos(Y), 0, sin(Y)],
                     [0, 1, 0],
                     [-sin(Y), 0, cos(Y)]]);
    Rz2 = np.asarray([[cos(Z2), -sin(Z2), 0],
                     [sin(Z2), cos(Z2), 0],
                     [0, 0, 1]]);
    return np.matmul(np.matmul(Rz1, Ry), Rz2)

class Marker:

    MARKER_SIZE = 0.045

    def __init__(self, id, rot, trans, roll=None,pitch=None,yaw=None):
        self.id = id
        if roll is not None:
            self.rotation = fromRPYtoRmat(roll,
                                          pitch,
                                          yaw)
        else:
            self.rotation = rot
        self.translation = np.asarray(trans)
        self.translation = self.translation.reshape((-1,1))



class Robot:

    def __init__(self, num_links):
        self.markers = {k: [] for k in range(num_links)}
        self.links = {}


    def addMarker(self,linkid, marker):
        self.markers[linkid].append(marker)

    def setBasePosition(self, base_rotation, base_translation):
        self.base_rotation = base_rotation
        self.base_translation = base_translation
        self.base_translation = self.base_translation.reshape((-1,1))

    def getBasePosition(self):
        return self.base_rotation, self.base_translation

    def addLink(self, link_id, d, a, alpha):
        self.links[link_id] = (d,a,radians(alpha))

    def getLink(self,link_id):
        return self.links[link_id]

    def getMarker(self, id):
        for (link, markers) in self.markers.items():
            for marker in markers:
                if id == marker.id:
                    return marker, link
        return None,None

    def getLinksCount(self):
        return len(self.links)