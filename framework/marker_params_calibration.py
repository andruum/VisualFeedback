import time

import scipy
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


def configuration_error(parameters, *args):
    gray_image = args[0]
    markers_origin = args[1]
    aruco_dict = args[2] #aruco dict
    cam = args[3]
    aruco_params = args[4]


    error = 0.0

    cam_matrix, cam_distortion = cam.getIntrinsics()

    corners, ids, rejectedImgPoints = cv.aruco.detectMarkers(gray_image,
                                                             aruco_dict,
                                                             cameraMatrix=cam_matrix,
                                                             distCoeff=cam_distortion,
                                                             parameters=aruco_params)

    rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(corners, parameters[0]/1000.0,
                                                         cam_matrix, cam_distortion)

    for (id, tvec) in zip(ids, tvecs):
        for marker in markers_origin:
            if id == marker.id:
                for (id_, tvec_) in zip(ids, tvecs):
                    if id != id_:
                        distance_obsv = np.linalg.norm(tvec-tvec_)
                        for marker_ in markers_origin:
                            if id != marker_.id:
                                distance_true = np.linalg.norm(marker.translation-marker_.translation)
                                error += abs(distance_true-distance_obsv)

    return error



class MarkerParamsCalibrator:

    def __init__(self, configuration_director, aruco_dict, aruco_params, init_value):
        self.configuration_director = configuration_director

        self.aruco_dict = cv.aruco.Dictionary_get(aruco_dict)
        self.aruco_params = aruco_params

        self.init_value = init_value*1000

        self.results = []


    def process(self, cam):

        # TODO
        # cons = ({'type': 'ineq', 'fun': lambda q: math.pi / 2 - abs(q[0])},
        #         {'type': 'ineq', 'fun': lambda q: math.pi/2 - abs(q[1]) })

        # cons = ({'type': 'ineq', 'fun': lambda q: math.pi / 2 - abs(q[0])})

        image, timestamp = cam.getImage()
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

        res = minimize(configuration_error, self.init_value,
                       args=(gray, self.configuration_director.markers_origin, self.aruco_dict, cam, self.aruco_params),
                       method='COBYLA',
                       tol=1e-6
                       # ,constraints=cons
                       )
        print(res.x)
        self.results.append(res.x)
        self.init_value = res.x


    def calculateResult(self):
        self.results = np.asarray(self.results)
        print("Mean:",self.results.mean(0))
