from math import degrees
from transforms3d.euler import mat2euler

from camera.camera import *
from robot.configuration_director import ConfigurationDirector
from robot.robotstate import RobotState
from framework.state_estimation import Estimator
from framework.visual_tracking import VisualTracking
import time

from utils.plotter import Plotter

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

    cam = UsbCamera("http://192.168.137.22:8080/video",'TecnoInf1080',5)
    # cam = FromVideo("20190514175240224380.avi",'TecnoInf1080',5)
    # cam = FromVideo("20190514160446832423.avi",'TecnoInf640',10)
    # cam = FromImage("./camera/configs/TECNO/ex4.jpg",'TECNO')

    conf_dir = ConfigurationDirector('kuka_zerolink_listbase')

    visualtrack = VisualTracking(conf_dir)
    visualtrack.addCamera(cam)
    estimator = Estimator(conf_dir)

    robot_state = RobotState(conf_dir.getRobotConf())

    maxtime = 0
    first_step = True
    enable_log = False

    if enable_log:
        initlog()

    plotter_pos = Plotter()
    plotter_angle = Plotter()
    plotter_q = Plotter()

    time_debug = False

    try:
        while True:

            visualtrack.sense(robot_state,debug=True)


            # campos, camrot = cam.getPosition()
            # if campos is not None:
            #     print(campos)
            #     data = campos.flatten().tolist()
            #     euler_cam = list(mat2euler(camrot))
            #     timev = time.time()
            #     data.append(timev)
            #     euler_cam.append(timev)
            #     plotter_pos.addData(data)
            #     plotter_angle.addData(euler_cam)

            start = time.time()

            estimator.sense(robot_state)

            estm = robot_state.configuration_estimation
            estm = [item for sublist in estm for item in sublist]
            timev = time.time()
            # del estm[0:-1]
            estm.append(timev)
            plotter_q.addData(estm)

            for i, q in enumerate(robot_state.configuration_estimation):
                print(i, degrees(q))



            if enable_log:
                log_coords(robot_state.configuration_estimation)


            robot_state = robot_state.clone()

            if time_debug:
                delay = time.time() - start
                if delay>maxtime and not first_step:
                    maxtime = delay
                first_step = False
                print("delay:",delay,"maxtime:",maxtime)

    finally:
        if enable_log:
            closeLog()
        # plotter_pos.plotData(0,20)
        # plotter_angle.plotData(0,20)
        plotter_q.plotData(0,200)