import json
import os
from robot.robot_configuration import Marker, Robot


class ConfigurationDirector():
    CFG_FOLDER = "./configs"

    def __init__(self, name_of_config=None):
        if name_of_config is None:
            return
        pathcfg = os.path.join(os.path.dirname(__file__),
                               ConfigurationDirector.CFG_FOLDER,
                               name_of_config + '.json')
        with open(pathcfg, "r") as cfg:
            jsonstr = cfg.read()

        config = json.loads(jsonstr)
        self.markers_origin = []
        for marker_dict in config['markers']:
            marker = Marker(marker_dict["id"],
                            marker_dict['translation'],
                            marker_dict['ZYZ'])
            self.markers_origin.append(marker)

        self.robots_conf = []
        for cfg_robot in config['robots']:
            robot_config = Robot(cfg_robot['name'])

            robot_config.setBasePosition(cfg_robot['base_translation'])
            robot_config.setBaseRotation(cfg_robot['base_ZYZ'])

            for link_cfg in cfg_robot['links']:
                robot_config.addLink(link_cfg['id'], link_cfg['d'],
                                     link_cfg['a'], link_cfg['alpha'],link_cfg["offset_q"])
                for marker_link_cfg in link_cfg['markers']:
                    marker_link = Marker(marker_link_cfg['id'],
                                         marker_link_cfg['translation'],
                                         marker_link_cfg['ZYZ'])
                    robot_config.addMarker(link_cfg['id'], marker_link)

            self.robots_conf.append(robot_config)

    def getRobotConf(self):
        return self.robots_conf[0]

    def create_example_config(self):

        marker_origin1 = Marker(5, [0, 0, 0], [0, 0, 0])
        marker_origin2 = Marker(12, [55 / 1000.0, 0, 0], [90, 0, 0])
        self.markers_origin = []
        self.markers_origin.append(marker_origin1)
        self.markers_origin.append(marker_origin2)

        robot_conf = Robot("Paper")
        robot_conf.setBasePosition([32 / 1000.0, 35 / 1000.0, 0.0])
        robot_conf.setBaseRotation([0.0, -90.0, 0.0])

        robot_conf.addLink(0, 0.0, 0.0, 0.0, 0.0)
        marker_link0 = Marker(6, [105 / 1000.0, -4 / 1000.0, 0], [-95, 0, 0])
        robot_conf.addMarker(0, marker_link0)

        robot_conf.addLink(1, 0.0, 190 / 1000.0, 0.0, 0.0)
        marker_link1 = Marker(7, [90 / 1000.0, -7 / 1000.0, 0], [-180, 0, 0])
        robot_conf.addMarker(1, marker_link1)

        self.robots_conf = []
        self.robots_conf.append(robot_conf)

    def dump_configs(self, filename):
        pathcfg = os.path.join(os.path.dirname(__file__),
                               ConfigurationDirector.CFG_FOLDER,
                               filename + '.json')
        print(pathcfg)
        cfg_dict = {}
        cfg_dict['markers'] = []
        for m in self.markers_origin:
            m_dict = {'id': m.id,
                      'translation': m.translation.tolist(),
                      'ZYZ': m.ZYZ}
            cfg_dict['markers'].append(m_dict)

        cfg_dict['robots'] = []
        for robotcfg in self.robots_conf:
            robot_dict = {'name': robotcfg.robot_name,
                          'base_translation': robotcfg.getBaseTransform()[1].tolist(),
                          'base_ZYZ': robotcfg.baseZYZ,
                          'links': []}

            for (link_id, link_cfg) in robotcfg.links.items():
                link_dict = {'id': link_id,
                             'markers': [],
                             'd': link_cfg['d'],
                             'a': link_cfg['a'],
                             'offset_q': link_cfg['offset_q'],
                             'alpha': link_cfg['alpha']}
                for m in link_cfg['markers']:
                    m_dict = {'id': m.id,
                              'translation': m.translation.tolist(),
                              'ZYZ': m.ZYZ}
                    link_dict['markers'].append(m_dict)
                robot_dict['links'].append(link_dict)

            cfg_dict['robots'].append(robot_dict)

        jsonstr = json.dumps(cfg_dict, indent=4)
        with open(pathcfg, 'w') as f:
            f.write(jsonstr)


if __name__ == '__main__':
    c = ConfigurationDirector()
    c.create_example_config()
    c.dump_configs('test2')
