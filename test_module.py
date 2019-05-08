from math import degrees
from camera.camera import *
from robot.configuration_director import ConfigurationDirector
from robot.robotstate import RobotState
from framework.state_estimation import Estimator
from framework.visual_tracking import VisualTracking
import time

if __name__ == '__main__':
    cam = UsbCamera("http://192.168.137.14:8080/video",'TecnoInf640',15)
    # cam = FromVideo("20190506182557089688.avi",'TecnoInf640',15)
    # cam = FromImage("./camera/configs/TECNO/ex4.jpg",'TECNO')

    conf_dir = ConfigurationDirector('test_paper')

    visualtrack = VisualTracking(conf_dir)
    visualtrack.addCamera(cam)
    estimator = Estimator(conf_dir)

    robot_state = RobotState(conf_dir.getRobotConf())

    maxtime = 0
    first_step = True


    time_debug = True

    while True:
        start = time.time()
        visualtrack.sense(robot_state,debug=True)


        # campos,_ = cam.getPosition()
        # if campos is not None:
            # print(campos)

        estimator.sense(robot_state)

        for i, q in enumerate(robot_state.configuration_estimation):
            print(i, degrees(q))

        robot_state = robot_state.clone()

        if time_debug:
            delay = time.time() - start
            if delay>maxtime and not first_step:
                maxtime = delay
            first_step = False
            print("delay:",delay,"maxtime:",maxtime)
