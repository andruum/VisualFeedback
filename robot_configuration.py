

class Marker:

    MARKER_SIZE = 0.0145

    def __init__(self, id, origin_rotation, origin_translation):
        self.id = id
        self.origin_rotation = origin_rotation
        self.origin_translation = origin_translation


class Robot:

    def __init__(self, num_links):
        self.markers = {k: [] for k in range(num_links)}
        self.links = {}


    def addMarker(self,linkid, marker):
        self.markers[linkid].append(marker)

    def setBasePosition(self,x,y,z):
        self.base_pos = [x,y,z]

    def addLink(self, link_id, transform_matrix):
        self.links[link_id] = transform_matrix