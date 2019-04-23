import cv2 as cv
from cv2 import aruco

from robot.robot_configuration import Marker

import numpy as np


class VisualTracking:

    def __init__(self, configuration_director):
        self.configuration_director = configuration_director
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters_create()
        self.cameras = []


    def addCamera(self, *cameras):
        for cam in cameras:
            self.cameras.append(cam)

    def estimateCamPosition(self, cam, observations):
        camera_tvec_res = []
        camera_rmat_res = []

        for (marker,rvec,tvec) in observations:

            rmat, _ = cv.Rodrigues(rvec)
            camera_rmat = np.transpose(rmat)
            tvec = tvec.reshape((-1, 1))
            camera_tvec = -np.matmul(camera_rmat, tvec)

            camera_rmat = np.matmul(marker.rotation, camera_rmat)
            camera_tvec = np.matmul(marker.rotation, camera_tvec) + marker.translation.reshape((-1, 1))

            camera_tvec_res.append(camera_tvec)
            camera_rmat_res.append(camera_rmat)
            # TODO calculate pose from several markers Kalman filter

        # if len(camera_tvec_res) > 1:
        #     error = camera_tvec_res[0][1]-camera_tvec_res[1][1]
        #     error = abs(error)
        #     print(error)

        if len(camera_tvec_res) == 0:
            cam.setPosition(None, None)
            return

        camera_tvec = [0, 0, 0]

        for r in camera_tvec_res:
            camera_tvec[0] += r[0] / len(camera_tvec_res)
            camera_tvec[1] += r[1] / len(camera_tvec_res)
            camera_tvec[2] += r[2] / len(camera_tvec_res)

        camera_rmat = camera_rmat_res[0]

        cam.setPosition(camera_tvec,camera_rmat)

    def sense(self, robot_state, debug = False):

        for cam in self.cameras:
            image,timestamp = cam.getImage()
            cam_matrix, cam_distortion = cam.getIntrinsics()

            gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,
                                                                  self.aruco_dict,
                                                                  cameraMatrix=cam_matrix,
                                                                  distCoeff=cam_distortion,
                                                                  parameters=self.aruco_params)

            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners,Marker.MARKER_SIZE,
                                                              cam_matrix,cam_distortion)

            if debug:
                frame_markers = aruco.drawDetectedMarkers(image.copy(), corners, ids)
                frame_markers = cv.resize(frame_markers, (800, 600))
                cv.imshow("aruco:", frame_markers)
                cv.waitKey(50)

            if ids is None:
                break

            origin_markers = []
            for marker in self.configuration_director.markers_origin:
                for (id, rvec, tvec) in zip(ids, rvecs, tvecs):
                    if marker.id == id:
                        observation_origin = (marker,rvec,tvec)
                        origin_markers.append(observation_origin)
                        np.delete(ids,np.argwhere(ids==id))
                        np.delete(rvecs,np.argwhere(rvecs==rvec))
                        np.delete(tvecs,np.argwhere(tvecs==tvec))
                        break

            self.estimateCamPosition(cam,origin_markers)

            robot_state.markers_observations.append((timestamp, ids, rvecs, tvecs))

        #do something with observations

        # origins = {k: [] for k in range(self.configuration_director.getRobotConf().getLinksCount())}

        for (cam,obsv) in zip(self.cameras,robot_state.markers_observations):

            (timestamp,ids, rvecs, tvecs) = obsv
            cam_translation,cam_rotation = cam.getPosition()

            if cam_translation is None or cam_rotation is None:
                break

            for (id, rvec, tvec) in zip(ids, rvecs, tvecs):
                marker, linkid = self.configuration_director.getRobotConf().getMarker(id)
                if marker is not None:
                    rmat, _ = cv.Rodrigues(rvec)
                    marker_rot = np.matmul(cam_rotation, rmat)
                    tvec = tvec.reshape((-1, 1))
                    marker_tvec = np.matmul(cam_rotation, tvec) + cam_translation

                    marker_rot_link = np.transpose(marker.rotation)
                    link_rot = np.matmul(marker_rot, marker_rot_link)

                    link_tvec = -np.matmul(link_rot, marker.translation)
                    link_tvec = link_tvec + marker_tvec

                    robot_state.axises_estimations[linkid] = (link_rot, link_tvec)

                    # TODO Kalman filter for markers, after calculate axises


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

                # TODO Kalman filter

        return origins

