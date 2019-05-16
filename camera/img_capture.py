import cv2
import os
from os import path

camera_name = "TecnoInf1080"

final_path = path.join("configs", camera_name, "config_images")
if not os.path.isdir(final_path):
    os.makedirs (final_path)

cam = cv2.VideoCapture("http://192.168.137.130:8080/video")

# cam.set(cv2.CAP_PROP_FPS, 30)
# cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1600)
# cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)

cv2.namedWindow("test")
img_counter = 0

while True:
    ret, frame = cam.read()
    resized = cv2.resize(frame,(640,480))
    cv2.imshow("test", resized)

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