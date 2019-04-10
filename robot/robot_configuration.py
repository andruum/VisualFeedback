

class Marker:

    MARKER_SIZE = 0.045

    def __init__(self, id, rot, trans):
        self.id = id
        self.rotation = rot
        self.translation = trans


class Robot:

    def __init__(self, num_links):
        self.markers = {k: [] for k in range(num_links)}
        self.links = {}


    def addMarker(self,linkid, marker):
        self.markers[linkid].append(marker)

    def setBasePosition(self, base_rotation, base_translation):
        self.base_rotation = base_rotation
        self.base_translation = base_translation

    def getBasePosition(self):
        return self.base_rotation, self.base_translation

    def addLink(self, link_id, d, a, alpha):
        self.links[link_id] = (d,a,alpha)

    def getLink(self,link_id):
        return self.links[link_id]

    def getMarker(self, id):
        for (link, marker) in self.markers.items():
            if id == marker.id:
                return marker, link
        return None

    def getLinksCount(self):
        return len(self.links)