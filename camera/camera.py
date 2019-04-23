import glob
import itertools
import json

import cv2 as cv
import numpy as np
import time
import os

class Camera:

    CAMERAS_FOLDER = "./configs"
    CALIBRATION_FILE_NAME = "calibration.json"

    def __init__(self, camera_name):
        self.camera_name = camera_name
        self.translation = None
        self.rotation = None

        path_to_calibration = os.path.join(os.path.dirname(__file__),
                                           Camera.CAMERAS_FOLDER,
                                           self.camera_name,
                                           Camera.CALIBRATION_FILE_NAME)

        self.exists = os.path.isfile(path_to_calibration)
        if self.exists:
            f = open(path_to_calibration,'r')
            jsonstr = f.read()
            f.close()
            config = json.loads(jsonstr)
            self.cam_matrix = np.asarray(config["cam_matrix"])
            self.cam_dist_mat = np.asarray(config["distortion"])


    def setPosition(self, translation, rotation):
        self.translation = translation
        self.rotation = rotation

    def getPosition(self):
        return self.translation,self.rotation

    def setIntrinsics(self, mtx, dst):
        self.cam_matrix = mtx
        self.cam_dist_mat = dst

    def dumpParams(self):
        dict = {"cam_matrix": self.cam_matrix.tolist(), "distortion": self.cam_dist_mat.tolist()}
        jsonfile = json.dumps(dict)

        path_to_calibration = os.path.join(os.path.dirname(__file__),
                                           Camera.CAMERAS_FOLDER,
                                           self.camera_name,
                                           Camera.CALIBRATION_FILE_NAME)

        f = open(path_to_calibration,'w')
        f.write(jsonfile)
        f.close()

    def getIntrinsics(self):
        if not self.exists:
            raise Exception("Calibration file doesn't exist!")
        else:
            return self.cam_matrix,self.cam_dist_mat

    def getImage(self):
        pass

    def getCameraFolder(self):
        path_to_folder = os.path.join(os.path.dirname(__file__),Camera.CAMERAS_FOLDER,self.camera_name)
        return path_to_folder


class UsbCamera(Camera):

    def __init__(self,device_id, camera_name):
        super().__init__(camera_name)
        self.cap = cv.VideoCapture(device_id)
        self.frame_width = int(self.cap.get(3))
        self.frame_height = int(self.cap.get(4))


    def getImage(self):
        ret, frame = self.cap.read()
        ts = time.time()
        return frame,ts

    def release(self):
        self.cap.release()

    def getWidthHeightFrame(self):
        return self.frame_width,self.frame_height


class FromImage(Camera):

    def __init__(self, image_path, camera_name, cycle = False):
        super().__init__(camera_name)
        isdir = os.path.isdir(image_path)
        if isdir:
            imgs = glob.glob(image_path+'/[0-1][0-9].*')
            self.images_paths = iter(imgs)
        else:
            self.images_paths = iter(list(image_path))
        if cycle:
            self.images_paths = itertools.cycle(self.images_paths)

    def getImage(self):
        image = cv.imread(next(self.images_paths))
        ts = time.time()
        return image,ts


class FromVideo(Camera):

    def __init__(self, video_path, camera_name):
        super().__init__(camera_name)
        video_path = os.path.join(self.getCameraFolder(),video_path)
        self.cap = cv.VideoCapture(video_path)

    def getImage(self):
        ret, frame = self.cap.read()
        if not ret:
            raise Exception("Video ended!")
        ts = time.time()
        return frame,ts