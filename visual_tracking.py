


class VisualTracking:


    def __init__(self, robot_params):
        self.robot = robot_params

    def addMarker(self, marker):
        self.markers[marker.link_id] = marker

    def senseScene(self, image, cam_position):


    def getPartsPosition(self):
        return {}