import time

from filterpy.kalman import MerweScaledSigmaPoints, UnscentedKalmanFilter
from pyquaternion import Quaternion
from scipy.optimize import minimize
import numpy as np
from numpy import cos,sin
from math import atan2
import cv2 as cv
from transforms3d.euler import mat2euler

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

def residual_x(a, b):
    y = a - b

    for i in range(0, len(y), 3):
        y[i] = normalize_angle(y[i])

    return y

def state_mean(sigmas, Wm):
    x = np.zeros(sigmas.shape[1])

    for i in range(0,len(x),3):
        sum_sin = np.sum(np.dot(np.sin(sigmas[:, i]), Wm))
        sum_cos = np.sum(np.dot(np.cos(sigmas[:, i]), Wm))
        x[i] = atan2(sum_sin, sum_cos)
        x[i+1] = np.sum(np.dot(sigmas[:, i+1], Wm))
        x[i+2] = np.sum(np.dot(sigmas[:, i+2], Wm))

    return x


def z_mean(sigmas, Wm):
    z_count = sigmas.shape[1]
    x = np.zeros(z_count)
    for z in range(0, z_count, 6):
        x[z] = np.sum(np.dot(sigmas[:, z], Wm))
        x[z + 1] = np.sum(np.dot(sigmas[:, z + 1], Wm))
        x[z + 2] = np.sum(np.dot(sigmas[:, z + 2], Wm))

        sum_sin = np.sum(np.dot(np.sin(sigmas[:, 3]), Wm))
        sum_cos = np.sum(np.dot(np.cos(sigmas[:, 3]), Wm))
        x[z + 3] = atan2(sum_sin, sum_cos)
        sum_sin = np.sum(np.dot(np.sin(sigmas[:, 4]), Wm))
        sum_cos = np.sum(np.dot(np.cos(sigmas[:, 4]), Wm))
        x[z + 4] = atan2(sum_sin, sum_cos)
        sum_sin = np.sum(np.dot(np.sin(sigmas[:, 5]), Wm))
        sum_cos = np.sum(np.dot(np.cos(sigmas[:, 5]), Wm))
        x[z + 5] = atan2(sum_sin, sum_cos)
    return x

def getMeasureVectorFromTrans(Tr):
    Rmat = Tr[0:3,0:3]
    t = Tr[0:3,3]
    euler = rotationMatrixToEulerAngles(Rmat)
    return [t[0],t[1],t[2],euler[0],euler[1],euler[2]]



def residual_h(a, b):
    y = a - b
    for i in range(3, len(y), 6):
        y[i] = normalize_angle(y[i])
        y[i + 1] = normalize_angle(y[i + 1])
        y[i + 2] = normalize_angle(y[i + 2])
    return y


def hx(x, camera, robot_configuration, observed_markers):
    nstates = int(len(x) / (SYSTEMORDER + 1))

    R_w_base, t_w_base = robot_configuration.getBaseTransform()
    T_w_base = fromRTtoTrans(R_w_base,t_w_base)

    Z = []

    T_w_l = T_w_base

    for i in range(nstates):
        d, a, al = robot_configuration.getLinkParams(i)
        T_pl_cl = getTransformMatrix(x[i*3],d,a,al)

        T_w_l = np.matmul(T_w_l,T_pl_cl)

        markers = robot_configuration.getMarkers(i)

        for m in markers:
            if m.id not in observed_markers:
                continue

            t_l_m = m.translation
            R_l_m = m.rotation
            T_l_m = fromRTtoTrans(R_l_m,t_l_m)
            T_w_m = np.matmul(T_w_l,T_l_m)

            t_w_c, R_w_c  = camera.getPosition()
            t_w_c = t_w_c.reshape((-1, 1))

            T_w_c = fromRTtoTrans(R_w_c , t_w_c)
            T_c_m = np.matmul(inverseTransform(T_w_c),T_w_m)

            zi = getMeasureVectorFromTrans(T_c_m)
            Z.extend(zi)

    return Z

def getMeasurementsVector(observations):
    observed_markers = []
    z = []
    for obsv in observations:
        (id,rvec,t) = obsv
        observed_markers.append(id)
        rmat, _ = cv.Rodrigues(rvec)
        t = t.reshape((-1, 1))
        euler = mat2euler(rmat)
        z.append([t[0],t[1],t[2],euler[0],euler[1],euler[2]])
    return z,observed_markers

def updateFilterDim(filter, new_dim_z):
    filter._dim_z = new_dim_z
    filter.R = np.eye(new_dim_z)*(0.02**2)


from numpy import linalg as la

def nearestPD(A):
    """Find the nearest positive-definite matrix to input

    A Python/Numpy port of John D'Errico's `nearestSPD` MATLAB code [1], which
    credits [2].

    [1] https://www.mathworks.com/matlabcentral/fileexchange/42885-nearestspd

    [2] N.J. Higham, "Computing a nearest symmetric positive semidefinite
    matrix" (1988): https://doi.org/10.1016/0024-3795(88)90223-6
    """

    B = (A + A.T) / 2
    _, s, V = la.svd(B)

    H = np.dot(V.T, np.dot(np.diag(s), V))

    A2 = (B + H) / 2

    A3 = (A2 + A2.T) / 2

    if isPD(A3):
        return A3

    spacing = np.spacing(la.norm(A))
    # The above is different from [1]. It appears that MATLAB's `chol` Cholesky
    # decomposition will accept matrixes with exactly 0-eigenvalue, whereas
    # Numpy's will not. So where [1] uses `eps(mineig)` (where `eps` is Matlab
    # for `np.spacing`), we use the above definition. CAVEAT: our `spacing`
    # will be much larger than [1]'s `eps(mineig)`, since `mineig` is usually on
    # the order of 1e-16, and `eps(1e-16)` is on the order of 1e-34, whereas
    # `spacing` will, for Gaussian random matrixes of small dimension, be on
    # othe order of 1e-16. In practice, both ways converge, as the unit test
    # below suggests.
    I = np.eye(A.shape[0])
    k = 1
    while not isPD(A3):
        mineig = np.min(np.real(la.eigvals(A3)))
        A3 += I * (-mineig * k**2 + spacing)
        k += 1

    return A3

def isPD(B):
    """Returns true when input is positive-definite, via Cholesky"""
    try:
        _ = la.cholesky(B)
        return True
    except la.LinAlgError:
        return False

from scipy.linalg import cholesky

def sqrt_method(x):
    try:
        return cholesky(x)
    except np.linalg.LinAlgError:
        return nearestPD(x)


def configuration_error(q, *args):
    observations = args[0] # (cam, observed_markers, positions)
    robot_configuration = args[1]

    R_w_base, t_w_base = robot_configuration.getBaseTransform()
    T_w_base = fromRTtoTrans(R_w_base, t_w_base)

    T_w_l = T_w_base

    error = 0

    for i,qi in enumerate(q):
        d, a, al = robot_configuration.getLinkParams(i)
        T_pl_cl = getTransformMatrix(qi, d, a, al)

        T_w_l = np.matmul(T_w_l, T_pl_cl)

        markers = robot_configuration.getMarkers(i)

        for m in markers:

            for obsv in observations:
                cam, observed_markers, positions = obsv

                if m.id not in observed_markers:
                    continue

                t_l_m = m.translation
                R_l_m = m.rotation
                T_l_m = fromRTtoTrans(R_l_m, t_l_m)
                T_w_m = np.matmul(T_w_l, T_l_m)

                t_w_c, R_w_c = cam.getPosition()
                t_w_c = t_w_c.reshape((-1, 1))

                T_w_c = fromRTtoTrans(R_w_c, t_w_c)
                T_c_m = np.matmul(inverseTransform(T_w_c), T_w_m)

                zi = getMeasureVectorFromTrans(T_c_m)

                idarray = observed_markers.index(m.id)
                z_obsv = positions[idarray]

                for x,y in zip(zi,z_obsv):
                    error += abs(x-y)
    return error


def estimate_configuration_minimization():
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

class Estimator:


    def __init__(self, configuration_director):
        self.configuration_director = configuration_director

        points = MerweScaledSigmaPoints(n=3*configuration_director.getRobotConf().getLinksCount(),
                                        alpha = .1, beta=2.,kappa=-1,sqrt_method=sqrt_method)

        self.ukf = UnscentedKalmanFilter(dim_x=3*configuration_director.getRobotConf().getLinksCount(),
                                         dim_z=6,
                                         fx=fx,
                                         hx=hx,
                                         dt = 0.1,
                                         points=points,
                                         x_mean_fn=state_mean,
                                         z_mean_fn=z_mean, residual_x=residual_x,
                                         residual_z=residual_h)

        self.ukf.Q = np.diag([0.001**2,0.005**2,0.010**2,]*configuration_director.getRobotConf().getLinksCount())
        self.update_time = time.time()


    def sense(self, robot_state):

        dt = time.time()-self.update_time

        for obsv in robot_state.markers_observations:

            (timestamp, cam, z) = obsv

            if len(z) == 0:
                continue

            z, observed_markers = getMeasurementsVector(z)

            updateFilterDim(self.ukf,len(z))

            self.ukf.predict(0.1)
            self.ukf.update(z, camera=cam,
                            robot_configuration=self.configuration_director.getRobotConf(),
                            observed_markers=observed_markers)

            print(self.ukf.x)

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



        # qinit = []
        # for i, RT in robot_state.axises_estimations.items():
        #     if len(RT) == 0:
        #         break
        #     else:
        #         if len(robot_state.configuration_prev)>i:
        #             qinit.append(robot_state.configuration_prev[i])
        #         else:
        #             qinit.append(0.0)
        #
        # if len(qinit) == 0:
        #     return False
        #
        # q0 = np.asarray(qinit)
        #
        # #
        # # cons = ({'type': 'ineq', 'fun': lambda q: math.pi / 2 - abs(q[0])},
        # #         {'type': 'ineq', 'fun': lambda q: math.pi/2 - abs(q[1]) })
        #
        # cons = ({'type': 'ineq', 'fun': lambda q: math.pi / 2 - abs(q[0])})
        # res = minimize(minimize_error_all, q0, args=(robot_state.axises_estimations, self.configuration_director.getRobotConf()),
        #                method='COBYLA',
        #                tol=1e-6,
        #                constraints=cons)
        #
        # robot_state.configuration_estimation = res.x

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
