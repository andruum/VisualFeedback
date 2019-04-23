import cv2
import datetime
import os
from camera import UsbCamera

camera = UsbCamera(0,"WebCam")

file_name = str(datetime.datetime.now()).replace(" ","").replace(".","").replace("-","").replace(":","")+".avi"
path_to_save = os.path.join(camera.getCameraFolder(),file_name)
print("Video will be saved to ", path_to_save)
out = cv2.VideoWriter(path_to_save,
                      cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'),
                      30,
                      camera.getWidthHeightFrame())

while (True):
    frame,time = camera.getImage()
    out.write(frame)
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
camera.release()
out.release()
cv2.destroyAllWindows()