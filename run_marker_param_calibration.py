from cv2 import aruco
from math import degrees
from transforms3d.euler import mat2euler

from camera.camera import *
from framework.marker_calibration import MarkerCalibrator
from framework.marker_params_calibration import MarkerParamsCalibrator
from robot.configuration_director import ConfigurationDirector
from robot.robot_configuration import Marker
from robot.robotstate import RobotState
from framework.state_estimation import Estimator
from framework.visual_tracking import VisualTracking
import time

from utils.plotter import Plotter



if __name__ == '__main__':

    # cam = UsbCamera("http://192.168.137.14:8080/video",'TecnoInf1080', 1)
    cam = UsbCamera(1, 'Logitech', 30)
    # cam = FromVideo("20190514175240224380.avi",'TecnoInf1080',5)
    # cam = FromVideo("20190514160446832423.avi",'TecnoInf640',10)
    # cam = FromImage("./camera/configs/TECNO/ex4.jpg",'TECNO')

    conf_dir = ConfigurationDirector('kuka_full_listbase')

    aruco_params = aruco.DetectorParameters_create()
    calibrator = MarkerParamsCalibrator(conf_dir, aruco.DICT_4X4_50, aruco_params, Marker.MARKER_SIZE)



    try:
        for i in range(20):
            calibrator.process(cam)
    finally:
        calibrator.calculateResult()