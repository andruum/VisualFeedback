import numpy as np
import cv2
import glob
from camera import FromImage

square_size = 20
# Define the chess board rows and columns
rows = 9
cols = 7

# Set the termination criteria for the corner sub-pixel algorithm
criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 30, 0.001)

# Prepare the object points: (0,0,0), (1,0,0), (2,0,0), ..., (6,5,0). They are the same for all images
objectPoints = np.zeros((rows * cols, 3), np.float32)
objectPoints[:, :2] = np.mgrid[0:rows, 0:cols].T.reshape(-1, 2)
objectPoints *= square_size

# Create the arrays to store the object points and the image points
objectPointsArray = []
imgPointsArray = []


camera = FromImage("./configs/Defender/config_images","Defender",cycle=False,load_configs=False)
while True:
    try:
        img,_ = camera.getImage()
    except StopIteration:
        break

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, (rows, cols), None)

    if ret:
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        objectPointsArray.append(objectPoints)
        imgPointsArray.append(corners)
        cv2.drawChessboardCorners(img, (rows, cols), corners, ret)


    # Display the image
    # rimg = cv2.resize(img,(500,700))
    # cv2.imshow('chess board', rimg)
    # cv2.waitKey(1000)

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objectPointsArray, imgPointsArray, gray.shape[::-1], None, None)

camera.resolution = gray.shape[::-1]
camera.setIntrinsics(mtx,dist)
camera.dumpParams()

error = 0
for i in range(len(objectPointsArray)):
    imgPoints, _ = cv2.projectPoints(objectPointsArray[i], rvecs[i], tvecs[i], mtx, dist)
    error += cv2.norm(imgPointsArray[i], imgPoints, cv2.NORM_L2) / len(imgPoints)
print("Total error: ", error / len(objectPointsArray))

