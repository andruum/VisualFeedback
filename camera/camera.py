import datetime
import glob
import itertools
import json
from threading import Thread

import cv2 as cv
import numpy as np
import time
import os


def threaded(fn):
    """To use as decorator to make a function call threaded.
    Needs import
    from threading import Thread"""

    def wrapper(*args, **kwargs):
        thread = Thread(target=fn, args=args, kwargs=kwargs)
        thread.start()
        return thread

    return wrapper


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
            with open(path_to_calibration, 'r') as f:
                jsonstr = f.read()
            config = json.loads(jsonstr)
            self.cam_matrix = np.asarray(config["cam_matrix"])
            self.cam_dist_mat = np.asarray(config["distortion"])

    def setPosition(self, translation, rotation):
        self.translation = translation
        self.rotation = rotation

    def getPosition(self):
        return self.translation, self.rotation

    def setIntrinsics(self, mtx, dst):
        self.cam_matrix = mtx
        self.cam_dist_mat = dst

    def dumpParams(self):
        dict = {"cam_matrix": self.cam_matrix.tolist(),
                "distortion": self.cam_dist_mat.tolist()}
        jsonfile = json.dumps(dict)

        path_to_calibration = os.path.join(os.path.dirname(__file__),
                                           Camera.CAMERAS_FOLDER,
                                           self.camera_name,
                                           Camera.CALIBRATION_FILE_NAME)

        with open(path_to_calibration, 'w') as f:
            f.write(jsonfile)

    def getIntrinsics(self):
        if not self.exists:
            raise Exception("Calibration file doesn't exist!")
        else:
            return self.cam_matrix, self.cam_dist_mat

    def getImage(self):
        pass

    def getCameraFolder(self):
        path_to_folder = os.path.join(os.path.dirname(__file__), Camera.CAMERAS_FOLDER, self.camera_name)
        return path_to_folder

    @threaded
    def recordVideo(self, length, name=None):
        pass


class UsbCamera(Camera):

    def __init__(self, device_id, camera_name, fps=30, resolution=(640, 480)):
        super().__init__(camera_name)
        self.cap = cv.VideoCapture(device_id)
        self.FPS = fps
        self.resolution = resolution
        self.cap.set(cv.CAP_PROP_FPS, self.FPS)
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, self.resolution[0])
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, self.resolution[1])

        self.frame_width = int(self.cap.get(cv.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv.CAP_PROP_FRAME_HEIGHT))

    def getImage(self):
        ret, frame = self.cap.read()
        ts = time.time()
        return frame, ts

    def release(self):
        self.cap.release()

    def getWidthHeightFrame(self):
        return self.frame_width, self.frame_height

    def recordVideo(self, length, name=None):
        if name is None:
            name = str(datetime.datetime.now()) \
                       .replace(" ", "") \
                       .replace(".", "") \
                       .replace("-", "") \
                       .replace(":", "") + ".avi"

        path_to_save = os.path.join(self.getCameraFolder(), name)
        print("Video will be saved to ", path_to_save)

        out = cv.VideoWriter(path_to_save,
                             cv.VideoWriter_fourcc('M', 'J', 'P', 'G'),
                             self.FPS,
                             self.getWidthHeightFrame())

        starttime = time.time()
        while time.time() < starttime + length:
            frame, _ = self.getImage()
            out.write(frame)
        out.release()


class FromImage(Camera):

    def __init__(self, image_path, camera_name, cycle=False):
        super().__init__(camera_name)
        isdir = os.path.isdir(image_path)
        if isdir:
            imgs = glob.glob(image_path + '/[0-1][0-9].*')
            self.images_paths = iter(imgs)
        else:
            self.images_paths = iter(list(image_path))
        if cycle:
            self.images_paths = itertools.cycle(self.images_paths)

    def getImage(self):
        image = cv.imread(next(self.images_paths))
        ts = time.time()
        return image, ts


class FromVideo(Camera):

    def __init__(self, video_path, camera_name):
        super().__init__(camera_name)
        video_path = os.path.join(self.getCameraFolder(), video_path)
        self.cap = cv.VideoCapture(video_path)

    def getImage(self):
        ret, frame = self.cap.read()
        if not ret:
            raise Exception("Video ended!")
        ts = time.time()
        return frame, ts
