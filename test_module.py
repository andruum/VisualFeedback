from math import degrees
from camera.camera import *
from robot.configuration_director import ConfigurationDirector
from robot.robotstate import RobotState
from framework.state_estimation import Estimator
from framework.visual_tracking import VisualTracking
import time

log_file = None

def initlog():
    file_name = str(datetime.datetime.now()) \
                    .replace(" ", "") \
                    .replace(".", "") \
                    .replace("-", "") \
                    .replace(":", "") + "_estimated.txt"
    global log_file
    log_file = open(file_name, "a+")

def log_coords(coords):
    coords = [item for sublist in coords for item in sublist]
    coords.append(time.time())
    log_file.write(str(coords))
    log_file.write("\n")

def closeLog():
    log_file.close()

if __name__ == '__main__':
    initlog()
    # cam = UsbCamera("http://192.168.137.56:8080/video",'TecnoInf640',10)
    cam = FromVideo("20190506182557089688.avi",'TecnoInf640',15)
    # cam = FromImage("./camera/configs/TECNO/ex4.jpg",'TECNO')

    conf_dir = ConfigurationDirector('kuka')

    visualtrack = VisualTracking(conf_dir)
    visualtrack.addCamera(cam)
    estimator = Estimator(conf_dir)

    robot_state = RobotState(conf_dir.getRobotConf())

    maxtime = 0
    first_step = True


    time_debug = False

    while True:

        visualtrack.sense(robot_state,debug=False)


        campos,_ = cam.getPosition()
        if campos is not None:
            print(campos)

        start = time.time()

        estimator.sense(robot_state)

        # for i, q in enumerate(robot_state.configuration_estimation):
        #     print(i, degrees(q))

        log_coords(robot_state.configuration_estimation)


        robot_state = robot_state.clone()

        if time_debug:
            delay = time.time() - start
            if delay>maxtime and not first_step:
                maxtime = delay
            first_step = False
            print("delay:",delay,"maxtime:",maxtime)

    closeLog()