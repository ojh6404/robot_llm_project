# -*- coding: utf-8 -*-

import rospy
import skrobot


class GotoAction(object):

    def __init__(self, robot_interface):
        self.robot_interface = robot_interface

    def goto(self, target=None, frame_id="map"):
        if target is None:
            # do nothing
            return
        return self.robot_interface.move_to(target, frame_id=frame_id)

    def __call__(self, target=None):
        return self.goto(target)


if __name__ == "__main__":
    import skrobot
    import skrobot_hsr
    from skrobot_hsr.ros.hsrb import HSRBROSRobotInterface

    shelf_front = skrobot.coordinates.Coordinates(
        pos=[6.87282599413, -15.3249358577, 0.0],
        rot=[0.604539666285, 0.0, 0.0, -0.796575038454],
    )
    kettle_front = skrobot.coordinates.Coordinates(
        pos=[7.65026867349, -14.5172523299, 0.0],
        rot=[0.999860089132, 0.0, 0.0, -0.0167272878904],
    )
    center_table = skrobot.coordinates.Coordinates(
        pos=[4.02303429754, -13.8952817604, 0.0],
        rot=[0.737792419137, 0.0, 0.0, -0.675027663332],
    )
    microwave_coords = skrobot.coordinates.Coordinates(
        pos=[6.53038740542, -12.5712576666, 0.0],
        rot=[0.729502853045, 0.0, 0.0, 0.683977768205],
    )

    r = skrobot_hsr.HSRB()
    ri = HSRBROSRobotInterface(r)
    r.angle_vector(ri.angle_vector())

    act = GotoAction(ri)
    act(microwave_coords)
    act(kettle_front)
