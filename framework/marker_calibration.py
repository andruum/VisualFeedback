import time

from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import MerweScaledSigmaPoints, UnscentedKalmanFilter, KalmanFilter
from pyquaternion import Quaternion
from scipy.optimize import minimize
import numpy as np
from numpy import cos, sin
from math import atan2
import cv2 as cv
from transforms3d.euler import mat2euler

from robot.robot_configuration import fromZYZtoRmat
from utils.utils import *


def getLinkPoisition(robot_state, robot_configuration, link_id):
    R_w_base, t_w_base = robot_configuration.getBaseTransform()
    T_w_base = fromRTtoTrans(R_w_base, t_w_base)

    T_w_l = T_w_base

    q = robot_state.configuration_estimated
    for i, qi in enumerate(q):
        if i > link_id:
            break
        d, a, al, offset_q = robot_configuration.getLinkParams(i)
        T_pl_cl = getTransformMatrix(qi + offset_q, d, a, al)

        T_w_l = np.matmul(T_w_l, T_pl_cl)

    return T_w_l

def configuration_error(marker_pos_inlink, *args):
    marker_observations = args[0]  # [(cam, rvec, tvec),]
    T_w_l = args[1] # T_w_l

    error = 0.0
    for obsv in marker_observations:
        (cam, rvec, tvec) = obsv

        t_l_m  = marker_pos_inlink[0:3]
        t_l_m = t_l_m.reshape((-1,1))
        ZYZ_l_m = marker_pos_inlink[3:6]
        R_l_m = fromZYZtoRmat(ZYZ_l_m)

        T_l_m = fromRTtoTrans(R_l_m, t_l_m)
        T_w_m = np.matmul(T_w_l, T_l_m)

        t_w_c, R_w_c = cam.getPosition()
        if t_w_c is None or R_w_c is None:
            continue
        t_w_c = t_w_c.reshape((-1, 1))

        T_w_c = fromRTtoTrans(R_w_c, t_w_c)
        T_c_m = np.matmul(inverseTransform(T_w_c), T_w_m)

        Rmat = T_c_m[0:3, 0:3]
        euler = rotationMatrixToEulerAngles(Rmat)

        z_expected = [*T_c_m[0:3, 3],*euler]

        rmat, _ = cv.Rodrigues(rvec)
        euler = mat2euler(rmat)
        z_obsv = [*tvec[0],*euler]

        for i in range(3):
            error += 100*abs(z_expected[i] - z_obsv[i])
            error += abs(normalize_angle(z_expected[i + 3] - z_obsv[i + 3]))/math.pi*180
    return error


def getVariableVector(marker):
    return [*marker.translation[:]]

class MarkerCalibrator:

    def __init__(self, configuration_director, marker_id):
        self.configuration_director = configuration_director
        self.marker_id = marker_id
        marker,self.link_id = self.configuration_director.getRobotConf().getMarker(marker_id)
        self.poses_init = np.asarray([*marker.translation,*marker.ZYZ])
        self.results = []


    def process(self, robot_state):

        if len(self.poses_init) == 0:
            return

        observations = []

        for obsv in robot_state.markers_observations:
            (timestamp, cam, z) = obsv
            for zi in z:
                if zi[0] == self.marker_id:
                    observations.append((cam, zi[1], zi[2]))
                    break

        T_w_l = getLinkPoisition(robot_state, self.configuration_director.getRobotConf(), self.link_id)

        # TODO
        # cons = ({'type': 'ineq', 'fun': lambda q: math.pi / 2 - abs(q[0])},
        #         {'type': 'ineq', 'fun': lambda q: math.pi/2 - abs(q[1]) })

        # cons = ({'type': 'ineq', 'fun': lambda q: math.pi / 2 - abs(q[0])})

        res = minimize(configuration_error, self.poses_init,
                       args=(observations, T_w_l),
                       method='COBYLA',
                       tol=1e-6
                       # ,constraints=cons
                       )
        self.results.append(res.x)
        print(res.x)


    def calculateResult(self):
        self.results = np.asarray(self.results[int(len(self.results)/2):-1])
        print("Mean:",self.results.mean(0))
