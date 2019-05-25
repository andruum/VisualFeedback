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


def fromZYZtoRmat(ZYZ):
    Z1 = radians(ZYZ[0])
    Y = radians(ZYZ[1])
    Z2 = radians(ZYZ[2])
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

    MARKER_SIZE = 0.04375

    def __init__(self, id, trans, ZYZrot):
        self.id = id
        self.rotation = fromZYZtoRmat(ZYZrot)
        self.translation = np.asarray(trans)
        self.translation = self.translation.reshape((-1,1))
        self.ZYZ = ZYZrot

class Robot:

    def __init__(self, robot_name = None):
        self.links = {}
        self.robot_name = robot_name
        self.baseZYZ = None
        self.base_rotation = None
        self.base_translation = None

    def addMarker(self, linkid, marker):
        if linkid not in self.links:
            self.links[linkid] = {}
            self.links[linkid]['markers'] = []
        self.links[linkid]['markers'].append(marker)

    def setBaseRotation(self,ZYZ):
        self.baseZYZ = ZYZ
        self.base_rotation = fromZYZtoRmat(ZYZ)

    def setBasePosition(self,base_translation):
        self.base_translation = np.asarray(base_translation)
        self.base_translation = self.base_translation.reshape((-1, 1))

    def getBaseTransform(self):
        return self.base_rotation, self.base_translation

    def addLink(self, link_id, d, a, alpha, offset_q):
        if link_id not in self.links:
            self.links[link_id] = {}
            self.links[link_id]['markers'] = []
        self.links[link_id]['d'] = d
        self.links[link_id]['a'] = a
        self.links[link_id]['alpha'] = radians(alpha)
        self.links[link_id]['offset_q'] = radians(offset_q)
        # self.links[link_id] = (d,a,radians(alpha))

    def getLinkParams(self,link_id):
        return self.links[link_id]['d'],self.links[link_id]['a'],self.links[link_id]['alpha'],self.links[link_id]['offset_q']

    def getMarker(self, id):
        for link_id in self.links.keys():
            for marker in self.links[link_id]['markers']:
                if id == marker.id:
                    return marker, link_id
        return None,None

    def getMarkers(self,linkid):
        return self.links[linkid]['markers']

    def isMarkerfromRobot(self, idmarker):
        for link_id in self.links.keys():
            for marker in self.links[link_id]['markers']:
                if idmarker == marker.id:
                    return True
        return False

    def getLinkIdByMarkerId(self, marker_id):
        for link_id in self.links.keys():
            for marker in self.links[link_id]['markers']:
                if marker_id == marker.id:
                    return link_id
        return link_id

    def getLinksCount(self):
        return len(self.links)