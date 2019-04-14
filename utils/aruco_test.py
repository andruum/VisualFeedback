from camera.camera import UsbCamera,FromImage
import cv2 as cv
from cv2 import aruco

cam = UsbCamera(0,'WebCam')
# cam = FromImage("../camera/configs/TECNO/ex4.jpg",'TECNO')
cam_matrix,cam_distortion = cam.getIntrinsics()


aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
aruco_params = aruco.DetectorParameters_create()

while True:
    image,timestamp = cam.getImage()

    # image = cv.resize(image, (800,600))

    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,
                                                          aruco_dict,
                                                          cameraMatrix=cam_matrix,
                                                          distCoeff=cam_distortion,
                                                          parameters=aruco_params)

    frame_markers = aruco.drawDetectedMarkers(image.copy(), corners, ids)
    frame_markers = cv.resize(frame_markers, (800, 600))
    cv.imshow("aruco:",frame_markers)
    cv.waitKey(50)