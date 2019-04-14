import cv2 as cv
from cv2 import aruco

from robot.robot_configuration import Marker

import numpy as np


class VisualTracking:

    def __init__(self, robot_params):
        self.robot = robot_params
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters_create()

    ### Returns positions of links in 3D
    def senseScene(self, image, cam_translation, cam_rotation, cam_matrix, cam_distortion, timestamp):
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,
                                                              self.aruco_dict,
                                                              parameters=self.aruco_params)

        rvecs, tvecs, trash = aruco.estimatePoseSingleMarkers(corners,
                                                              Marker.MARKER_SIZE,
                                                              cam_matrix,
                                                              cam_distortion)

        origins = {k: [] for k in range(self.robot.getLinksCount())}

        for (id, rvec, tvec) in zip(ids, rvecs, tvecs):
            marker, linkid = self.robot.getMarker(id)
            if marker is not None:
                rmat, _ = cv.Rodrigues(rvec)
                marker_rot = np.matmul(cam_rotation,rmat)
                tvec = tvec.reshape((-1, 1))
                marker_tvec = np.matmul(cam_rotation,tvec) + cam_translation

                marker_rot_link = np.transpose(marker.rotation)
                link_rot = np.matmul(marker_rot,marker_rot_link)

                link_tvec = -np.matmul(link_rot,marker.translation)
                link_tvec = link_tvec + marker_tvec

                origins[linkid] = (link_rot, link_tvec)

        return origins