import cv2
import datetime
import os
from camera import UsbCamera, cv

if __name__ == '__main__':

    # camera = UsbCamera(0,"WebCam")
    # camera = UsbCamera("http://192.168.137.112:8080/video",'TecnoInf640', 5)

    # cap = cv.VideoCapture("http://192.168.137.112:8080/video")
    cap = cv.VideoCapture(1)
    # cap.open(cv.CAP_DSHOW)
    # cap.set(cv.CAP_PROP_FOURCC,cv.VideoWriter_fourcc('M','J','P','G'))
    # cap.set(cv.CAP_PROP_FRAME_WIDTH,1920)
    # cap.set(cv.CAP_PROP_FRAME_HEIGHT,1080)
    # cap.set(cv.CAP_PROP_FPS,30)
    cap.set(cv.CAP_PROP_FOCUS,5)
    print(cap.get(cv.CAP_PROP_FRAME_WIDTH),cap.get(cv.CAP_PROP_FRAME_HEIGHT))
    file_name = str(datetime.datetime.now()).replace(" ","").replace(".","").replace("-","").replace(":","")+".avi"
    path_to_save = os.path.join("./",file_name)
    print("Video will be saved to ", path_to_save)
    out = cv2.VideoWriter(path_to_save,
                          cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'),
                          30,
                          (int(cap.get(cv.CAP_PROP_FRAME_WIDTH)),int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))))

    while (True):
        ret, frame = cap.read()
        out.write(frame)
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    out.release()
    cv2.destroyAllWindows()
