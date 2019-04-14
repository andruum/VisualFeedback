from math import degrees

from camera.camera_position_estimator import CameraPostionEstimator
from camera.camera import *
from robot.robot_configuration import *


# Markers for position estimation
# marker_origin1 = Marker(4,None,[0,0,0],0,0,0)
# marker_origin2 = Marker(3,None,[53/1000.0,0,0],0,0,0)
# marker_origin3 = Marker(2,None,[0.0,-52.5/1000.0,0],-90,0,0)
from state_estimation import Estimator
from visual_tracking import VisualTracking

marker_origin1 = Marker(5,None,[0,0,0],0,0,0)
marker_origin2 = Marker(12,None,[55/1000.0,0,0],90,0,0)
# marker_origin3 = Marker(2,None,[0.0,-52.5/1000.0,0],-90,0,0)

cam_pose_estimator = CameraPostionEstimator()
cam_pose_estimator.addMarker(marker_origin1)
cam_pose_estimator.addMarker(marker_origin2)
# cam_pose_estimator.addMarker(marker_origin3)


cam = UsbCamera(0,'WebCam')
# cam = FromImage("./camera/configs/TECNO/ex4.jpg",'TECNO')

robot_conf = Robot(2)
Rbase = fromZYZtoRmat(90.0,-90.0,0.0)
Tbase = np.asarray([32/1000.0,35/1000.0,0.0])
robot_conf.setBasePosition(Rbase,Tbase)

robot_conf.addLink(0,0.0,0.0,0.0)
robot_conf.addLink(1,0.0,190/1000.0,0.0)

marker_link0 = Marker(6,None,[105/1000.0,-4/1000.0,0],-95,0,0)
marker_link1 = Marker(7,None,[90/1000.0,-7/1000.0,0],-180,0,0)

robot_conf.addMarker(0,marker_link0)
robot_conf.addMarker(1,marker_link1)

visualtrack = VisualTracking(robot_conf)

estimator = Estimator(robot_conf)

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