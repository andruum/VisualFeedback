import numpy as np
import cv2, PIL
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd


aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)


nx = 4
ny = 3
pages = 2

file_name = "markers_4_4_50_"

for p in range(pages):
    fig = plt.figure()
    for i in range(1, nx*ny+1):
        ax = fig.add_subplot(ny,nx, i)
        img = aruco.drawMarker(aruco_dict, i+p*nx*ny, 700*2)
        plt.imshow(img, cmap = mpl.cm.gray, interpolation = "nearest")
        ax.axis("off")
    plt.savefig("../_data/"+str(file_name)+str(p)+".pdf")
    plt.clf()

