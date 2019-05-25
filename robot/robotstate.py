

class RobotState():

    def __init__(self):
        self.markers_observations = []
        self.configuration_prev = []
        self.configuration_estimated = []

    def reset(self):
        self.configuration_prev = self.configuration_estimated
        self.configuration_estimated = []
        self.markers_observations = []