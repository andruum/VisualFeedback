

from visual_tracking import VisualTracking
from camera import *
from robot_configuration import *




cam = FromImage("/path")
cam = FromVideo("/path")
cam = UsbCamera(0)

num_links = 2
robot_conf = Robot(num_links)

marker1 = Marker()

visualtrack = VisualTracking(cam)

