import cv2
import os
from os import path

camera_name = "WebCam"

final_path = path.join("configs", camera_name, "config_images")
if not os.path.isdir(final_path):
    os.makedirs (final_path)

cam = cv2.VideoCapture(0)
cv2.namedWindow("test")
img_counter = 0

while True:
    ret, frame = cam.read()
    cv2.imshow("test", frame)
    if not ret:
        break
    k = cv2.waitKey(1)

    if k%256 == 27:
        # ESC pressed
        print("Escape hit, closing...")
        break
    elif k%256 == 32:
        # SPACE pressed
        img_counter_str = str(img_counter)
        img_counter_str = img_counter_str.zfill(2)
        img_name = "{}/{}.png".format(final_path,img_counter_str)
        cv2.imwrite(img_name, frame)
        print("{} written!".format(img_name))
        img_counter += 1

cam.release()

cv2.destroyAllWindows()