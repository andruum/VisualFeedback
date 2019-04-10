from camera.camera import UsbCamera,FromImage
import cv2 as cv
from cv2 import aruco

cam = UsbCamera(0,'WebCam')
cam = FromImage("/home/andruum/projects/VisualFeedback/camera/configs/TECNO/ex7.jpg",'TECNO')

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
aruco_params = aruco.DetectorParameters_create()

while True:
    image,timestamp = cam.getImage()

    image = cv.resize(image, (1500,1000))

    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,
                                                          aruco_dict,
                                                          parameters=aruco_params)

    frame_markers = aruco.drawDetectedMarkers(image.copy(), corners, ids)


    cv.imshow("aruco:",frame_markers)
    cv.waitKey(50)