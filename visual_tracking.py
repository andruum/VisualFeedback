import cv2 as cv
from cv2 import aruco
import matplotlib.pyplot as plt

from robot_configuration import Marker


class VisualTracking:


    def __init__(self, robot_params):
        self.robot = robot_params
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters_create()



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

        for (id, rvec, tvec) in zip(ids,rvecs, tvecs):
            marker, link = self.robot.getMarker(id)
            if marker is not None:
                rmat = cv.Rodrigues(rvec)
                marker_rot = cam_rotation*rmat
                marker_tvec = cam_rotation*tvec+cam_translation


                #TODO calculate position of link origin in world frame




    def getPartsPosition(self):
        return {}