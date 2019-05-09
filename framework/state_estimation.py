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

from utils.utils import *


def getMeasureVectorFromTrans(Tr):
    Rmat = Tr[0:3, 0:3]
    t = Tr[0:3, 3]
    euler = rotationMatrixToEulerAngles(Rmat)
    return [t[0], t[1], t[2], euler[0], euler[1], euler[2]]


def getMeasurement(rvec, t):
    rmat, _ = cv.Rodrigues(rvec)
    t = t.reshape((-1, 1))
    euler = mat2euler(rmat)
    return [t[0], t[1], t[2], euler[0], euler[1], euler[2]]


def getMeasurementsVector(observations):
    observed_markers = []
    z = []
    for obsv in observations:
        (id, rvec, t) = obsv
        observed_markers.append(id)
        rmat, _ = cv.Rodrigues(rvec)
        t = t.reshape((-1, 1))
        euler = mat2euler(rmat)
        z.append([t[0], t[1], t[2], euler[0], euler[1], euler[2]])
    return z, observed_markers


def configuration_error(q, *args):
    observations = args[0]  # (cam, observed_markers, positions)
    robot_configuration = args[1]

    R_w_base, t_w_base = robot_configuration.getBaseTransform()
    T_w_base = fromRTtoTrans(R_w_base, t_w_base)

    T_w_l = T_w_base

    error = 0

    for i, qi in enumerate(q):
        d, a, al, offset_q = robot_configuration.getLinkParams(i)
        T_pl_cl = getTransformMatrix(qi+offset_q, d, a, al)

        T_w_l = np.matmul(T_w_l, T_pl_cl)

        markers = robot_configuration.getMarkers(i)

        for m in markers:

            for obsv in observations:
                (timestamp, cam, z) = obsv

                for (id, rvec, t) in z:
                    if m.id == id:

                        t_l_m = m.translation
                        R_l_m = m.rotation
                        T_l_m = fromRTtoTrans(R_l_m, t_l_m)
                        T_w_m = np.matmul(T_w_l, T_l_m)

                        t_w_c, R_w_c = cam.getPosition()
                        t_w_c = t_w_c.reshape((-1, 1))

                        T_w_c = fromRTtoTrans(R_w_c, t_w_c)
                        T_c_m = np.matmul(inverseTransform(T_w_c), T_w_m)

                        zi = getMeasureVectorFromTrans(T_c_m)

                        z_obsv = getMeasurement(rvec, t)

                        for i in range(3):
                            error += abs(zi[i] - z_obsv[i])
                            error += abs(normalize_angle(zi[i + 3] - z_obsv[i + 3]))
    return error


def estimate_configuration_minimization(robot_state, robot_conf):

    qinit = []

    for i in range(robot_conf.getLinksCount()):
        if len(robot_state.configuration_prev) > i:
            qinit.append(robot_state.configuration_prev[i])
        else:
            qinit.append(0.0)


    if len(qinit) == 0:
        return None

    q0 = np.asarray(qinit)

    # TODO
    # cons = ({'type': 'ineq', 'fun': lambda q: math.pi / 2 - abs(q[0])},
    #         {'type': 'ineq', 'fun': lambda q: math.pi/2 - abs(q[1]) })

    # cons = ({'type': 'ineq', 'fun': lambda q: math.pi / 2 - abs(q[0])})

    res = minimize(configuration_error, q0,
                   args=(robot_state.markers_observations, robot_conf),
                   method='COBYLA',
                   tol=1e-6
                   # ,constraints=cons
                   )

    return res.x

def getQ(dt, links):
    Q_var = 0.005**2
    Q = Q_discrete_white_noise(dim=2, dt=dt, var=Q_var)
    Qfull = np.zeros((2*links,2*links))
    for i in range(links):
        Qfull[i*2,i*2] = Q[0,0]
        Qfull[i*2+1,i*2] = Q[1,0]
        Qfull[i*2,i*2+1] = Q[0,1]
        Qfull[i*2+1,i*2+1] = Q[1,1]
    return Qfull

def getF(dt, links):
    Qfull = np.zeros((2 * links, 2 * links))
    for i in range(links):
        Qfull[i * 2, i * 2] = 1
        Qfull[i * 2 + 1, i * 2] = 0
        Qfull[i * 2, i * 2 + 1] = dt
        Qfull[i * 2 + 1, i * 2 + 1] = 1
    return Qfull

def getH(links):
    Qfull = np.zeros((links, 2 * links))
    for i in range(links):
        Qfull[i, i*2] = 1
    return Qfull

class Estimator:

    def __init__(self, configuration_director):
        self.configuration_director = configuration_director

        self.kf = KalmanFilter(dim_x=2*configuration_director.getRobotConf().getLinksCount(),
                               dim_z=configuration_director.getRobotConf().getLinksCount())

        self.kf.F = getF(0.05, configuration_director.getRobotConf().getLinksCount())
        self.kf.Q = getQ(0.05, configuration_director.getRobotConf().getLinksCount())
        self.kf.H = getH(configuration_director.getRobotConf().getLinksCount())
        self.kf.R = np.diag([0.001 ** 2] * configuration_director.getRobotConf().getLinksCount())

        self.lastupdate = None

    def sense(self, robot_state):

        q_m = estimate_configuration_minimization(robot_state,self.configuration_director.getRobotConf())

        if self.lastupdate is None:
            if q_m is None:
                return
            for i in range(len(q_m)):
                self.kf.x[i*2] = q_m[i]
            self.lastupdate = time.time()
        else:
            dt = time.time()-self.lastupdate
            self.lastupdate = time.time()

            F = getF(dt,self.configuration_director.getRobotConf().getLinksCount())
            Q = getQ(dt,self.configuration_director.getRobotConf().getLinksCount())
            self.kf.predict(F=F,Q=Q)
            if q_m is not None:
                self.kf.update(q_m)

        robot_state.configuration_estimation = [self.kf.x[i] for i in range(0,2*self.configuration_director.getRobotConf().getLinksCount(),2)]
