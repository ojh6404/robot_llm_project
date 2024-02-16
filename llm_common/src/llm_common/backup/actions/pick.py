# -*- coding: utf-8 -*-

import numpy as np
import tf
import hsrb_interface
from skrobot.coordinates import Coordinates
from jsk_robocup_common.coord_utils import matrix2quaternion

class PickObjectAction(object):

    def __init__(self, whole_body, gripper, omni_base):
        self.whole_body = whole_body
        self.gripper = gripper
        self.omni_base = omni_base

    def pick(self, target: Coordinates=None, ref_frame_id: str='map', force: float=1.0, pre_grasp: float=0.1, post_grasp: float=0.2, open_distance: float=1.2, upper_offset: float=0.00):
        """
        :param Coordinates target: Target coordinate of the pick operation. The orientation of this coordinate also determines the direction of the reaching.
        :param str ref_frame_id: Frame ID of the reference for the target's coordinate representation.
        :param float force: Force to close the gripper.
        :param float pre_grasp: Distance between grasping position and pre-grasp position when reaching.
        :param float post_grasp: Distance between grasping position and post-grasp position after grasp.
        :param float open_distance: Distance to first opening of gripper.
        :param float upper_offset: Upper offset for pre-grasp and post-grasp.
        """
        if target is None:
            # do nothing
            return

        # pre_grasp
        self.gripper.set_distance(open_distance)
        target.translate([upper_offset, 0, -1.0 * pre_grasp], "local")
        self.whole_body.move_end_effector_pose(
            hsrb_interface.geometry.Pose(
                pos=np.array(target.worldpos()),
                ori=matrix2quaternion(target.rotation)),
            ref_frame_id=ref_frame_id)

        # reaching
        target.translate([-1.0 * upper_offset, 0, pre_grasp], "local")
        self.whole_body.move_end_effector_pose(
            hsrb_interface.geometry.Pose(
                pos=np.array(target.worldpos()),
                ori=matrix2quaternion(target.rotation)),
            ref_frame_id=ref_frame_id)

        # grasp object
        self.gripper.apply_force(force)

        # pick up object
        target.translate([upper_offset, 0, -1.0 * post_grasp], "local")
        self.whole_body.move_end_effector_pose(
            hsrb_interface.geometry.Pose(
                pos=np.array(target.worldpos()),
                ori=matrix2quaternion(target.rotation)),
            ref_frame_id=ref_frame_id)
        return

    def __call__(self, target=None):
        return self.pick(target)


if __name__ == '__main__':
    import hsrb_interface

    print("Initialize robot interface")
    robot = hsrb_interface.Robot()
    from hsrb_interface import settings
    settings._SETTINGS[u'trajectory']['action_timeout'] = 1
    whole_body = robot.try_get('whole_body')
    settings._SETTINGS[u'trajectory']['action_timeout'] = 30.0
    omni_base = robot.try_get('omni_base')
    gripper = robot.try_get('gripper')

    dummy_target = Coordinates(
        pos=[0.47365640148061416, 0.07646567549141614, 0.11689634540078984],
        rot=[-1.1161014323551877e-05, 0.9979444148772441, 0.06408528958065665, -0.00014265381319705783])

    act = PickObjectAction(whole_body, gripper, omni_base)
    act.pick(target=dummy_target, ref_frame_id="base_footprint")
