



class RobotState():

    def __init__(self,robot_configuration=None):
        self.markers_observations = []
        # if robot_configuration is not None:
        #     self.axises_estimations = {k: [] for k in range(robot_configuration.getLinksCount())}
        # else:
        #     self.axises_estimations = None
        self.configuration_prev = []
        self.configuration_estimation = []

    def clone(self):
        newstate = RobotState()
        # newstate.axises_estimations = {k: [] for k in range(len(self.axises_estimations))}
        newstate.configuration_prev = self.configuration_estimation
        return newstate


