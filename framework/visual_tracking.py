import cv2 as cv
from cv2 import aruco
from math import atan2
from transforms3d.euler import mat2euler, euler2mat

from robot.robot_configuration import Marker
import numpy as np
from filterpy.kalman import KalmanFilter, UnscentedKalmanFilter, MerweScaledSigmaPoints
from filterpy.common import Q_discrete_white_noise
from pyquaternion import Quaternion

from utils.utils import *


def getMeasurementVector(Tr):
    Rmat = Tr[0:3, 0:3]
    t = Tr[0:3, 3]
    euler = mat2euler(Rmat)
    return [t[0], t[1], t[2], euler[0], euler[1], euler[2]]


def Hx(x, markers):
    camera_tvec = np.array(x[0:3])
    camera_tvec = camera_tvec.reshape((-1, 1))
    camera_rmat = euler2mat(*x[3:6])
    T_w_c = fromRTtoTrans(camera_rmat, camera_tvec)
    T_c_w = inverseTransform(T_w_c)

    Z = []
    for m in markers:
        T_w_m = fromRTtoTrans(m.rotation, m.translation)
        T_c_m = np.matmul(T_c_w, T_w_m)
        Z.extend(getMeasurementVector(T_c_m))
    return Z


def residual_h(a, b):
    y = a - b
    for i in range(3, len(y), 6):
        y[i] = normalize_angle(y[i])
        y[i + 1] = normalize_angle(y[i + 1])
        y[i + 2] = normalize_angle(y[i + 2])
    return y


def fx(x, dt):
    return x


def residual_x(a, b):
    y = a - b
    y[3] = normalize_angle(y[3])
    y[4] = normalize_angle(y[4])
    y[5] = normalize_angle(y[5])
    return y


def state_mean(sigmas, Wm):
    x = np.zeros(6)

    x[0] = np.sum(np.dot(sigmas[:, 0], Wm))
    x[1] = np.sum(np.dot(sigmas[:, 1], Wm))
    x[2] = np.sum(np.dot(sigmas[:, 2], Wm))

    sum_sin = np.sum(np.dot(np.sin(sigmas[:, 3]), Wm))
    sum_cos = np.sum(np.dot(np.cos(sigmas[:, 3]), Wm))
    x[3] = atan2(sum_sin, sum_cos)
    sum_sin = np.sum(np.dot(np.sin(sigmas[:, 4]), Wm))
    sum_cos = np.sum(np.dot(np.cos(sigmas[:, 4]), Wm))
    x[4] = atan2(sum_sin, sum_cos)
    sum_sin = np.sum(np.dot(np.sin(sigmas[:, 5]), Wm))
    sum_cos = np.sum(np.dot(np.cos(sigmas[:, 5]), Wm))
    x[5] = atan2(sum_sin, sum_cos)

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


def createKalmanfilterCam(camera):
    points = MerweScaledSigmaPoints(n=6, alpha=.00001, beta=2, kappa=0,
                                    subtract=residual_x)

    f = UnscentedKalmanFilter(dim_x=6, dim_z=6, dt=1, hx=Hx,
                              points=points, fx=fx, x_mean_fn=state_mean,
                              z_mean_fn=z_mean, residual_x=residual_x,
                              residual_z=residual_h)

    f.x = np.array([0, 0, 0, 0, 0, 0])

    f.P[0, 0] = 0.003 ** 2
    f.P[1, 1] = 0.003 ** 2
    f.P[2, 2] = 0.003 ** 2
    f.P[3, 3] = 0.1 ** 2
    f.P[4, 4] = 0.1 ** 2
    f.P[5, 5] = 0.1 ** 2

    f.R[0, 0] = 0.003 ** 2
    f.R[1, 1] = 0.003 ** 2
    f.R[2, 2] = 0.003 ** 2
    f.R[3, 3] = 1 ** 2
    f.R[4, 4] = 1 ** 2
    f.R[5, 5] = 1 ** 2

    f.Q *= 0.000333 ** 2
    return f


def estimateCamPosition(cam, filter, observations):
    if len(observations) == 0:
        campos, camrot = cam.getPosition()
        if campos is not None and camrot is not None:
            filter.predict()
        else:
            return

    for (marker, rvec, tvec) in observations:
        rmat, _ = cv.Rodrigues(rvec)
        tvec = tvec.reshape((-1, 1))

        campos, camrot = cam.getPosition()
        if campos is None and camrot is None:
            camera_rmat = np.transpose(rmat)
            camera_tvec = -np.matmul(camera_rmat, tvec)
            camera_rmat = np.matmul(marker.rotation, camera_rmat)
            camera_tvec = np.matmul(marker.rotation, camera_tvec) + marker.translation.reshape((-1, 1))
            cam.setPosition(camera_tvec, camera_rmat)
            euler_cam = mat2euler(camera_rmat)
            filter.x = np.array([camera_tvec[0], camera_tvec[1], camera_tvec[2],
                                 euler_cam[0], euler_cam[1], euler_cam[2]])
            continue

        euler = mat2euler(rmat)
        # print(euler)
        z = np.asarray([tvec[0],
                        tvec[1],
                        tvec[2],
                        euler[0],
                        euler[1],
                        euler[2]])
        filter.predict()
        filter.update(z, markers=[marker])

    camera_tvec = np.array([*filter.x[0:3]])
    camera_rmat = euler2mat(*filter.x[3:6])
    # print(*filter.x[3:6])
    cam.setPosition(camera_tvec, camera_rmat)


class VisualTracking:

    def __init__(self, configuration_director):
        self.configuration_director = configuration_director
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters_create()
        self.aruco_params.cornerRefinementMethod = aruco.CORNER_REFINE_APRILTAG
        # self.aruco_params.cornerRefinementWinSize = 5
        # self.aruco_params.cornerRefinementMinAccuracy = 0.05
        # self.aruco_params.adaptiveThreshWinSizeMin = 3
        # self.aruco_params.adaptiveThreshWinSizeMax = 64
        # self.aruco_params.adaptiveThreshWinSizeStep = 5
        self.cameras = []
        self.camera_filters = []
        self.observationsz = None

    def addCamera(self, *cameras):
        for cam in cameras:
            self.cameras.append(cam)
            f = createKalmanfilterCam(cam)
            self.camera_filters.append(f)

    def sense(self, robot_state, debug=False):

        for (cam, filter) in zip(self.cameras, self.camera_filters):
            image, timestamp = cam.getImage()
            cam_matrix, cam_distortion = cam.getIntrinsics()

            gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,
                                                                  self.aruco_dict,
                                                                  cameraMatrix=cam_matrix,
                                                                  distCoeff=cam_distortion,
                                                                  parameters=self.aruco_params)

            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, Marker.MARKER_SIZE,
                                                              cam_matrix, cam_distortion)


            if debug:
                frame_markers = aruco.drawDetectedMarkers(image.copy(), corners, ids)
                if rvecs is not None:
                    for rvec,tvec in zip(rvecs,tvecs):
                        aruco.drawAxis(frame_markers,cam_matrix,cam_distortion,rvec,tvec,0.01)
                frame_markers = cv.resize(frame_markers, (800, 600))

                cv.imshow("aruco:", frame_markers)
                cv.waitKey(50)
                # cv.imwrite("example_markers.png",frame_markers)

            if ids is None:
                continue

            origin_markers = []
            for marker in self.configuration_director.markers_origin:
                for (id, rvec, tvec) in zip(ids, rvecs, tvecs):
                    if marker.id == id:
                        observation_origin = (marker, rvec, tvec)
                        origin_markers.append(observation_origin)
                        np.delete(ids, np.argwhere(ids == id))
                        np.delete(rvecs, np.argwhere(rvecs == rvec))
                        np.delete(tvecs, np.argwhere(tvecs == tvec))
                        break

            estimateCamPosition(cam, filter, origin_markers)

            if ids is None:
                continue
            z = []
            for (id, rvec, tvec) in zip(ids, rvecs, tvecs):
                if self.configuration_director.getRobotConf().isMarkerfromRobot(id):
                    z.append((id, rvec, tvec))

            robot_state.markers_observations.append((timestamp, cam, z))