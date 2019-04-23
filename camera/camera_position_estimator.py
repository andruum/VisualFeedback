import cv2 as cv
from cv2 import aruco
from robot.robot_configuration import Marker
import numpy as np
import math

class CameraPostionEstimator:

    def __init__(self):
        self.markers = []
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters_create()

    def addMarker(self, *markers):
        for marker in markers:
            self.markers.append(marker)

    def estimatePosition(self, image, cam_matrix, cam_distortion):
        # image, _ = camera.getImage()
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,
                                                              self.aruco_dict,
                                                              cameraMatrix=cam_matrix,
                                                              distCoeff=cam_distortion,
                                                              parameters=self.aruco_params)


        # cam_matrix,cam_distortion = camera.getIntrinsics()

        rvecs, tvecs, trash = aruco.estimatePoseSingleMarkers(corners,
                                                              Marker.MARKER_SIZE,
                                                              cam_matrix,
                                                              cam_distortion)
        frame_markers = aruco.drawDetectedMarkers(image.copy(), corners, ids)
        frame_markers = cv.resize(frame_markers, (800, 600))
        cv.imshow("aruco:", frame_markers)
        cv.waitKey(50)


        if ids is None:
            return None, None

        camera_tvec_res = []
        camera_rmat_res = []

        for (id, rvec, tvec) in zip(ids, rvecs, tvecs):
            for marker in self.markers:
                if marker.id == id:
                    # return tvec, rvec
                    rmat,_ = cv.Rodrigues(rvec)
                    camera_rmat = np.transpose(rmat)
                    tvec = tvec.reshape((-1, 1))
                    camera_tvec = -np.matmul(camera_rmat,tvec)

                    camera_rmat = np.matmul(marker.rotation,camera_rmat)
                    camera_tvec = np.matmul(marker.rotation,camera_tvec)+marker.translation.reshape((-1, 1))
                    # break
                    camera_tvec_res.append(camera_tvec)
                    camera_rmat_res.append(camera_rmat)
                    #TODO calculate pose from several markers Kalman filter

        # if len(camera_tvec_res) > 1:
        #     error = camera_tvec_res[0][1]-camera_tvec_res[1][1]
        #     error = abs(error)
        #     print(error)


        if len(camera_tvec_res) == 0:
            return None,None

        camera_tvec = [0,0,0]

        for r in camera_tvec_res:
            camera_tvec[0] += r[0]/len(camera_tvec_res)
            camera_tvec[1] += r[1]/len(camera_tvec_res)
            camera_tvec[2] += r[2]/len(camera_tvec_res)

        camera_rmat = camera_rmat_res[0]
        #Todo convert to quartenion

        return camera_tvec,camera_rmat