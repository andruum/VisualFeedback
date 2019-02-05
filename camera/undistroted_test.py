import cv2 as cv
from cv2 import aruco

from camera import UsbCamera

cam = UsbCamera(0,'WebCam')

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
aruco_params = aruco.DetectorParameters_create()

while True:
    image,timestamp = cam.getImage()

    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

    mtx, dist = cam.getIntrinsics()
    h, w = gray.shape[:2]

    newCameraMtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    undistortedImg = cv.undistort(gray, mtx, dist, None, newCameraMtx)

    # Crop the undistorted image
    x, y, w, h = roi
    undistortedImg = undistortedImg[y:y + h, x:x + w]

    cv.imshow("aruco:",undistortedImg)
    cv.waitKey(50)