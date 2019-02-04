import cv2 as cv



class Camera:

    def __init__(self):
        self.markers = []

    def calibrate(self):
        pass

    def getPosition(self):
        image = self.getImage()

        #todo detect markers and calculate position

        return None,None

    def getIntrinsics(self):
        return None,None

    def getImage(self):
        pass

    def addPositionMarker(self,marker):
        self.markers.append(marker)



class UsbCamera(Camera):

    def __init__(self):
        Camera.__init__(self)
        self.cap = cv.VideoCapture(0)

    def getImage(self):
        ret, frame = self.cap.read()
        return frame


class FromImage(Camera):
    pass

class FromVideo(Camera):
    pass