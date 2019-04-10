import cv2 as cv
import numpy as np
import time

import os

class Camera:

    def __init__(self, camera_name):
        config = np.load(
            os.path.join(os.path.dirname(__file__),
            './configs/{}/calibration.npz'.format(camera_name)))
        self.cam_matrix = config['mtx']
        self.cam_dist_mat = config['dist']

    def getIntrinsics(self):
        return self.cam_matrix,self.cam_dist_mat

    def getImage(self):
        pass


class UsbCamera(Camera):

    def __init__(self,device_id, camera_name):
        Camera.__init__(self,camera_name)
        self.cap = cv.VideoCapture(device_id)

    def getImage(self):
        ret, frame = self.cap.read()
        ts = time.time()
        return frame,ts


class FromImage(Camera):

    def __init__(self, image_path, camera_name):
        Camera.__init__(self,camera_name)
        self.image = cv.imread(image_path)

    def getImage(self):
        ts = time.time()
        return self.image,ts

class FromVideo(Camera):

    def __init__(self,video_path, camera_name):
        Camera.__init__(self,camera_name)
        self.cap = cv.VideoCapture(video_path)

    def getImage(self):
        ret, frame = self.cap.read()
        ts = time.time()
        return frame,ts