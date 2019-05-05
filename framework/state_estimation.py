from filterpy.kalman import MerweScaledSigmaPoints, UnscentedKalmanFilter
from pyquaternion import Quaternion
from scipy.optimize import minimize
import numpy as np
from numpy import cos,sin
import math

from utils.utils import *


def minimize_error_all(q,*args):
    parts = args[0]
    configuration = args[1]

    rotation_base, pos_base = configuration.getBaseTransform()
    tr_base = fromRTtoTrans(rotation_base,pos_base)

    error = 0
    for i,qi in enumerate(q):
        d, a, al = configuration.getLinkParams(i)

        tr_part = fromRTtoTrans(parts[i][0],parts[i][1])
        error_mat =  abs(np.matmul(tr_base,getTransformMatrix(qi,d,a,al)) - tr_part)
        error += np.mean(error_mat)

    return error

def movement(x0, v0, a, dt):
    return x0 + v0*dt+0.5*a*dt*dt

def velocity(v0, a, dt):
    return v0+a*dt

SYSTEMORDER = 2

def fx(x, dt):
    nstates = int(len(x)/(SYSTEMORDER+1))

    for i in range(nstates):
        x[i] = movement(x[i],x[nstates+i],x[nstates*2+i],dt)
        x[nstates+i] = velocity(x[nstates+i],x[nstates*2+i],dt)
    ##TODO Add limits for angles
    return x



def getMeasureVectorFromTrans(Tr):
    Rmat = Tr[0:3,0:3]
    t = Tr[0:3,3]
    euler = rotationMatrixToEulerAngles(Rmat)
    return [t[0],t[1],t[2],euler[0],euler[1],euler[2]]






def hx(x, camera, robot_configuration, observed_markers):
    nstates = int(len(x) / (SYSTEMORDER + 1))

    R_w_base, t_w_base = robot_configuration.getBaseTransform()
    T_w_base = fromRTtoTrans(R_w_base,t_w_base)

    Z = []

    T_w_l = T_w_base

    for i in range(nstates):
        d, a, al = robot_configuration.getLinkParams(i)
        T_pl_cl = getTransformMatrix(x[i],d,a,al)

        T_w_l = np.matmul(T_w_l,T_pl_cl)

        markers = robot_configuration.getMarkers(i)

        for m in markers:
            if m.id not in observed_markers:
                continue

            t_l_m = m.translation
            R_l_m = m.rotation
            T_l_m = fromRTtoTrans(R_l_m,t_l_m)
            T_w_m = np.matmul(T_w_l,T_l_m)

            R_w_c , t_w_c = camera.getPosition()

            T_w_c = fromRTtoTrans(R_w_c , t_w_c)
            T_c_m = np.matmul(inverseTransform(T_w_c),T_w_m)

            zi = getMeasureVectorFromTrans(T_c_m)
            Z.extend(zi)

    return Z

class Estimator:


    def __init__(self, configuration_director):
        self.configuration_director = configuration_director

        points = MerweScaledSigmaPoints(n=configuration_director.getRobotConf().getLinksCount(),
                                        alpha = .1, beta=2.,kappa=-1)

        self.ukf = UnscentedKalmanFilter(dim_x=3*configuration_director.getRobotConf().getLinksCount(),
                                         dim_z=7,
                                         fx=fx,
                                         hx=hx,
                                         dt = 0.1,
                                         points=points)


    def sense(self, robot_state):

        for obsv in robot_state.markers_observations:

            (timestamp, cam, z) = obsv

            if len(z) == 0:
                continue

            self.ukf._dim_z = 7*len(z)

            self.ukf #TODO find out how to average quaternions

            # cam_translation, cam_rotation = cam.getPosition()
            # if cam_translation is None or cam_rotation is None:
            #     break
            #
            # for (id, rvec, tvec) in zip(ids, rvecs, tvecs):
            #     marker, linkid = self.configuration_director.getRobotConf().getMarker(id)
            #     if marker is not None:
            #         rmat, _ = cv.Rodrigues(rvec)
            #         marker_rot = np.matmul(cam_rotation, rmat)
            #         tvec = tvec.reshape((-1, 1))
            #         marker_tvec = np.matmul(cam_rotation, tvec) + cam_translation
            #
            #         marker_rot_link = np.transpose(marker.rotation)
            #         link_rot = np.matmul(marker_rot, marker_rot_link)
            #
            #         link_tvec = -np.matmul(link_rot, marker.translation)
            #         link_tvec = link_tvec + marker_tvec
            #
            #         robot_state.axises_estimations[linkid] = (link_rot, link_tvec)



        qinit = []
        for i, RT in robot_state.axises_estimations.items():
            if len(RT) == 0:
                break
            else:
                if len(robot_state.configuration_prev)>i:
                    qinit.append(robot_state.configuration_prev[i])
                else:
                    qinit.append(0.0)

        if len(qinit) == 0:
            return False

        q0 = np.asarray(qinit)

        #
        # cons = ({'type': 'ineq', 'fun': lambda q: math.pi / 2 - abs(q[0])},
        #         {'type': 'ineq', 'fun': lambda q: math.pi/2 - abs(q[1]) })

        cons = ({'type': 'ineq', 'fun': lambda q: math.pi / 2 - abs(q[0])})
        res = minimize(minimize_error_all, q0, args=(robot_state.axises_estimations, self.configuration_director.getRobotConf()),
                       method='COBYLA',
                       tol=1e-6,
                       constraints=cons)

        robot_state.configuration_estimation = res.x

        return True

    def senseParts(self,parts):
        qinit = []
        for i,RT in parts.items():
            if len(RT) == 0:
                break
            else:
                qinit.append(0.0)

        if len(qinit)==0:
            return []
        q0 = np.asarray(qinit)

        #
        # cons = ({'type': 'ineq', 'fun': lambda q: math.pi / 2 - abs(q[0])},
                #         {'type': 'ineq', 'fun': lambda q: math.pi/2 - abs(q[1]) })

        cons = ({'type': 'ineq', 'fun': lambda q: math.pi / 2 - abs(q[0])})
        res = minimize(minimize_error_all, q0, args=(parts,self.configuration_director.getRobotConf()),
                       method='COBYLA',
                       tol=1e-6,
                       constraints = cons)
        return res.x
