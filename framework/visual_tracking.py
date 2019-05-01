import cv2 as cv
from cv2 import aruco

from robot.robot_configuration import Marker

import numpy as np

from filterpy.kalman import KalmanFilter

from filterpy.common import Q_discrete_white_noise
from pyquaternion import Quaternion


class VisualTracking:

    def __init__(self, configuration_director):
        self.configuration_director = configuration_director
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters_create()
        self.cameras = []
        self.camera_filters = []

    def addCamera(self, *cameras):
        for cam in cameras:
            self.cameras.append(cam)
            f = KalmanFilter(dim_x=7, dim_z=7)
            f.x = np.zeros((7, 1))
            f.F = np.eye(7)
            f.H = np.eye(7)
            f.P *= 1.
            f.R *= 1
            f.Q *= 0.05
            self.camera_filters.append(f)

    def estimateCamPosition(self, cam, filter, observations):
        if len(observations) == 0:
            cam.setPosition(None, None)
            return

        for (marker, rvec, tvec) in observations:

            rmat, _ = cv.Rodrigues(rvec)
            camera_rmat = np.transpose(rmat)
            tvec = tvec.reshape((-1, 1))
            camera_tvec = -np.matmul(camera_rmat, tvec)

            camera_rmat = np.matmul(marker.rotation, camera_rmat)
            camera_tvec = np.matmul(marker.rotation, camera_tvec) + marker.translation.reshape((-1, 1))

            quat = Quaternion(matrix=camera_rmat)

            z = np.asarray([camera_tvec[0],
                            camera_tvec[1],
                            camera_tvec[2],
                            quat.x,
                            quat.y,
                            quat.z,
                            quat.w])

            campos,camrot = cam.getPosition()
            if campos is None or camrot is None:
                filter.x = z
                continue

            filter.predict()
            filter.update(z)

        camera_tvec = filter.x[0:3]
        camera_quat_filtered = Quaternion(w=filter.x[6], x=filter.x[3], y=filter.x[4], z=filter.x[5])
        camera_rmat = camera_quat_filtered.rotation_matrix

        cam.setPosition(camera_tvec, camera_rmat)

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
                frame_markers = cv.resize(frame_markers, (800, 600))
                cv.imshow("aruco:", frame_markers)
                cv.waitKey(50)

            if ids is None:
                break

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

            self.estimateCamPosition(cam, filter, origin_markers)

            robot_state.markers_observations.append((timestamp, ids, rvecs, tvecs))

        # do something with observations

        # origins = {k: [] for k in range(self.configuration_director.getRobotConf().getLinksCount())}

        for (cam, obsv) in zip(self.cameras, robot_state.markers_observations):

            (timestamp, ids, rvecs, tvecs) = obsv
            cam_translation, cam_rotation = cam.getPosition()

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
                marker_rot = np.matmul(cam_rotation, rmat)
                tvec = tvec.reshape((-1, 1))
                marker_tvec = np.matmul(cam_rotation, tvec) + cam_translation

                marker_rot_link = np.transpose(marker.rotation)
                link_rot = np.matmul(marker_rot, marker_rot_link)

                link_tvec = -np.matmul(link_rot, marker.translation)
                link_tvec = link_tvec + marker_tvec

                origins[linkid] = (link_rot, link_tvec)

                # TODO Kalman filter

        return origins
