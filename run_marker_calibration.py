from math import degrees
from transforms3d.euler import mat2euler

from camera.camera import *
from framework.marker_calibration import MarkerCalibrator
from robot.configuration_director import ConfigurationDirector
from robot.robotstate import RobotState
from framework.state_estimation import Estimator
from framework.visual_tracking import VisualTracking
import time

from utils.plotter import Plotter



if __name__ == '__main__':

    # cam = UsbCamera("http://192.168.137.14:8080/video",'TecnoInf1080', 5)
    cam = FromVideo("experiment_2/static_zeros.avi", 'TecnoInf1080', 10)
    # cam = FromVideo("20190514160446832423.avi",'TecnoInf640',10)

    conf_dir = ConfigurationDirector('kuka_full_listbase')

    visualtrack = VisualTracking(conf_dir)
    visualtrack.addCamera(cam)
    calibrator = MarkerCalibrator(conf_dir, 10)

    robot_state = RobotState()
    robot_state.configuration_estimated = [0.0, 0.0, 0.0, 0.0, 0.0]

    try:
        for i in range(20):
            visualtrack.sense(robot_state, debug=True)
            calibrator.process(robot_state)
    finally:
        calibrator.calculateResult()