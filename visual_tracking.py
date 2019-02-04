import cv2 as cv
from cv2 import aruco
import matplotlib.pyplot as plt

from robot_configuration import Marker


class VisualTracking:


    def __init__(self, robot_params):
        self.robot = robot_params
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters_create()

    def senseScene(self, image, cam_translation, cam_rotation, cam_matrix, cam_distortion):
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        frame_markers = aruco.drawDetectedMarkers(image.copy(), corners, ids)

        #rvecs, tvecs, trash = aruco.estimatePoseSingleMarkers(corners, Marker.MARKER_SIZE, cam_matrix, cam_distortion)

        plt.figure()
        plt.imshow(frame_markers)
        plt.show()

    def getPartsPosition(self):
        return {}