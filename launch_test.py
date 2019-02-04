from visual_tracking import VisualTracking
from camera import *
from robot_configuration import *


# Markers for position estimation
marker_origin1 = Marker(1,None,None)
marker_origin2 = Marker(2,None,None)
marker_origin3 = Marker(3,None,None)
marker_origin4 = Marker(4,None,None)



cam = UsbCamera()
cam.addPositionMarker(marker_origin1)
cam.addPositionMarker(marker_origin2)
cam.addPositionMarker(marker_origin3)
cam.addPositionMarker(marker_origin4)

visualtrack = VisualTracking(None)

while True:
    image = cam.getImage()
    translation,rotation = cam.getPosition()
    cammatrix, cam_dist = cam.getIntrinsics()
    visualtrack.senseScene(image,translation,rotation,cammatrix,cam_dist)
