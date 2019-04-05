import cv2 as cv
from cv2 import aruco
from robot_configuration import Marker
import numpy as np

class CameraPostionEstimator:

    def __init__(self):
        self.markers = []
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters_create()

    def addMarker(self, marker):
        self.markers.append(marker)

    def estimatePosition(self, image, cam_matrix, cam_distortion):
        # image, _ = camera.getImage()
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,
                                                              self.aruco_dict,
                                                              parameters=self.aruco_params)


        # cam_matrix,cam_distortion = camera.getIntrinsics()

        rvecs, tvecs, trash = aruco.estimatePoseSingleMarkers(corners,
                                                              Marker.MARKER_SIZE,
                                                              cam_matrix,
                                                              cam_distortion)



        if ids is None:
            return None, None

        camera_tvec = None
        camera_rmat = None

        for (id, rvec, tvec) in zip(ids, rvecs, tvecs):
            for marker in self.markers:
                if marker.id == id:
                    rmat,_ = cv.Rodrigues(rvec)
                    camera_rmat = np.transpose(rmat)
                    camera_tvec = -np.dot(camera_rmat,np.transpose(tvec))
                    break
                    #TODO calculate pose from several markers

        return camera_tvec,camera_rmat