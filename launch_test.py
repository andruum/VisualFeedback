from camera.camera_position_estimator import CameraPostionEstimator
from visual_tracking import VisualTracking
from camera.camera import *
from robot_configuration import *


# Markers for position estimation
marker_origin1 = Marker(2,None,None)


cam_pose_estimator = CameraPostionEstimator()
cam_pose_estimator.addMarker(marker_origin1)


cam = UsbCamera(0,'WebCam')

# visualtrack = VisualTracking(None)

while True:
    image,timestamp = cam.getImage()
    cammatrix, cam_dist = cam.getIntrinsics()
    translation,rotation = cam_pose_estimator.estimatePosition(image,cammatrix,cam_dist)
    if translation is not None:
        print(translation[2])
    # visualtrack.senseScene(image,translation,rotation,cammatrix,cam_dist,timestamp)

