import numpy as np
import cv2, PIL
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd

if __name__ == '__main__':

    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)


    nx = 3
    ny = 2
    pages = 1
    file_name = "markerss_4_4_50_"

    for p in range(pages):
        fig = plt.figure()
        for i in range(1, nx*ny+1):
            ax = fig.add_subplot(ny,nx, i)
            img = aruco.drawMarker(aruco_dict, i+p*nx*ny, 700)
            square_size = int(img.shape[0]/6.0)
            img = cv2.copyMakeBorder(img, square_size, square_size, square_size, square_size, cv2.BORDER_CONSTANT, value=[255,255,255])
            # cv2.rectangle(img, (0,0), (square_size,square_size), [0,0,0], -1)
            # cv2.rectangle(img, (img.shape[0]-square_size,0), (img.shape[0],square_size), [0,0,0], -1)
            # cv2.rectangle(img, (0,img.shape[1]-square_size), (square_size,img.shape[1]), [0,0,0], -1)
            # cv2.rectangle(img, (img.shape[0]-square_size,img.shape[1]-square_size), (img.shape[0],img.shape[1]), [0,0,0], -1)

            plt.imshow(img, cmap = mpl.cm.gray, interpolation = "nearest")
            # rect = mpl.patches.Rectangle((0,0), 1000, 1000)
            # ax.add_patch(rect)
            ax.axis("off")

        # plt.show()
        plt.savefig("../_data/"+str(file_name)+str(p)+".pdf")
        plt.clf()

