from math import degrees
from camera.camera_position_estimator import CameraPostionEstimator
from camera.camera import *
from robot.configuration_director import ConfigurationDirector
from framework.state_estimation import Estimator
from framework.visual_tracking import VisualTracking


if __name__ == '__main__':
    cam = UsbCamera(0,'WebCam')
    # cam = FromImage("./camera/configs/TECNO/ex4.jpg",'TECNO')

    conf_dir = ConfigurationDirector('test')
    cam_pose_estimator = CameraPostionEstimator()
    cam_pose_estimator.addMarker(*conf_dir.markers_origin)

    visualtrack = VisualTracking(conf_dir.robots_conf)
    estimator = Estimator(conf_dir.robots_conf)

    while True:
        image,timestamp = cam.getImage()
        cammatrix, cam_dist = cam.getIntrinsics()
        translation,rotation = cam_pose_estimator.estimatePosition(image,cammatrix,cam_dist)
        if translation is not None:
            # print(translation[0])
            res = visualtrack.senseScene(image,translation,rotation,cammatrix,cam_dist,timestamp)
            if len(res) != 0:
                # print(res[0][1])
                resq = estimator.senseParts(res)
                for i,q in enumerate(resq):
                    print(i,degrees(q))