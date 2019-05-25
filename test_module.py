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

    # cam = UsbCamera("http://192.168.137.13:8080/video",'TecnoInf1080', 10)
    # cam = FromVideo("20190517180051292409.avi",'TecnoInf1080',5)
    # cam = FromVideo("20190517181713696754.avi",'TecnoInf1080',5)
    cam = FromVideo("experiment_2/zero_link.avi", 'TecnoInf1080', 10)

    conf_dir = ConfigurationDirector('kuka_full_listbase')

    visualtrack = VisualTracking(conf_dir)
    visualtrack.addCamera(cam)
    estimator = Estimator(conf_dir)

    robot_state = RobotState()

    maxtime = 0
    first_step = True
    enable_log = False

    if enable_log:
        initlog()

    plotter_pos = Plotter()
    plotter_angle = Plotter()
    plotter_q = Plotter()

    time_debug = True

    try:
        for i in range(1000):
            start = time.time()

            visualtrack.sense(robot_state,debug=True)


            campos, camrot = cam.getPosition()
            if campos is not None:
                print(campos)
            #     data = campos.flatten().tolist()
            #     euler_cam = list(mat2euler(camrot))
            #     timev = time.time()
            #     data.append(timev)
            #     euler_cam.append(timev)
            #     plotter_pos.addData(data)
            #     plotter_angle.addData(euler_cam)



            estimator.sense(robot_state)

            estm = robot_state.configuration_estimated
            estm = [item for sublist in estm for item in sublist]
            timev = time.time()
            # del estm[0:-1]
            estm.append(timev)
            plotter_q.addData(estm)

            for i, q in enumerate(robot_state.configuration_estimated):
                print(i, degrees(q))



            if enable_log:
                log_coords(robot_state.configuration_estimated)


            robot_state.reset()

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