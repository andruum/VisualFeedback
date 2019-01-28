

class Marker:

    def __init__(self, id, origin_rotation, origin_translation, link_id):
        self.id = id
        self.link_id = link_id
        self.origin_rotation = origin_rotation
        self.origin_translation = origin_translation

class Robot:

    def __init__(self, num_links):
        self.markers = {k: [] for k in range(num_links)}
        self.links = {}


    def addMarker(self, marker):
        self.markers[marker.link_id].append(marker)

    def setBasePosition(self,x,y,z):
        self.base_pos = [x,y,z]

    def addLink(self, link_id, transform_matrix):
        self.links[link_id] = transform_matrix