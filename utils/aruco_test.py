from camera.camera import UsbCamera,FromImage
import cv2 as cv
from cv2 import aruco

# cam = UsbCamera(0,'WebCam')
cam = UsbCamera(1,'Defender')
# cam = FromImage("../camera/configs/TECNO/ex1.jpg",'TECNO')
cam_matrix,cam_distortion = cam.getIntrinsics()


aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
aruco_params = aruco.DetectorParameters_create()

import time



while True:
    start = time.time()
    image,timestamp = cam.getImage()

    # image = cv.resize(image, (800,600))

    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,
                                                          aruco_dict,
                                                          # cameraMatrix=cam_matrix,
                                                          # distCoeff=cam_distortion,
                                                          parameters=aruco_params)

    frame_markers = aruco.drawDetectedMarkers(image.copy(), corners, ids)
    frame_markers = cv.resize(frame_markers, (800, 600))
    cv.imshow("aruco:",frame_markers)
    cv.waitKey(50)

    end = time.time()
    print(end - start)