#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# You can get cuttent pose of robot by $ rosrun jsk_robocup_common now-pose

import hsrb_interface
import numpy as np
import tf
from skrobot.coordinates import Coordinates
from jsk_robocup_common.coord_utils import matrix2quaternion

class PoseChange(object):

    def __init__(self, whole_body, gripper):
        self.whole_body = whole_body
        self.gripper = gripper

    def look_with_head(self, head_pan_joint: float=0.0, head_tilt_joint: float=-0.86, arm_lift_joint: float=0.0):
        if arm_lift_joint > 0.0:
            self.whole_body.move_to_joint_positions({'arm_flex_joint': 0.0, 'arm_roll_joint': 0.0, 'head_pan_joint': 0.0, 'head_tilt_joint': 0.0, 'wrist_flex_joint': -1.5699986694517207, 'wrist_roll_joint': 0.0})
            self.whole_body.move_to_joint_positions({'arm_lift_joint': arm_lift_joint})
            self.whole_body.move_to_joint_positions({'head_pan_joint': head_pan_joint, 'head_tilt_joint': head_tilt_joint})
        else:
            self.whole_body.move_to_joint_positions({'arm_flex_joint': 0.0, 'arm_lift_joint': arm_lift_joint, 'arm_roll_joint': -1.5699999993825218, 'wrist_flex_joint': -1.5699986694517207, 'wrist_roll_joint': 0.0002087858536721221, 'head_pan_joint': head_pan_joint, 'head_tilt_joint': head_tilt_joint})
        return

    def look_with_hand(self, target: Coordinates=None, ref_frame_id: str='map', distance: float=0.4):
        if target is None: # default pose
            whole_body.move_to_joint_positions({'arm_flex_joint': -0.9299687544868802, 'arm_lift_joint': 0.0015562783630354406, 'arm_roll_joint': -0.11920613613947317, 'head_pan_joint': -5.855961609846361e-06, 'head_tilt_joint': -0.5000008260019606, 'wrist_flex_joint': -1.8336200039870336, 'wrist_roll_joint': 0.251613702398501})
            # whole_body.move_to_joint_positions({'arm_flex_joint': -1.333329135807412, 'arm_lift_joint': 0.061926168425348815, 'arm_roll_joint': -0.043770999382521714, 'head_pan_joint': 1.6404440894568495e-06, 'head_tilt_joint': -9.534554957779662e-07, 'wrist_flex_joint': -1.220572669451721, 'wrist_roll_joint': -0.07303921414632786})
            return
        else: # closer look at the recognized target with hand camera
            target.translate([0, 0, -1.0 * distance], "local")
            self.whole_body.move_end_effector_pose(
                hsrb_interface.geometry.Pose(
                    pos=np.array(target.worldpos()),
                    ori=matrix2quaternion(target.rotation)),
                ref_frame_id=ref_frame_id)
            return

    def look_hand_with_head(self):
        # whole_body.move_to_joint_positions({'arm_flex_joint': -1.1036927544868802, 'arm_lift_joint': 0.00016368781302191718, 'arm_roll_joint': 3.242328863860527, 'head_pan_joint': 0.0, 'head_tilt_joint': -0.6, 'wrist_flex_joint': -1.1364380039870334, 'wrist_roll_joint': -1.721469297601499})
        whole_body.move_to_joint_positions({'arm_flex_joint': -1.1033087544868803, 'arm_lift_joint': 0.00016368781302191718, 'arm_roll_joint': 3.242328863860527, 'head_pan_joint': 0.0, 'head_tilt_joint': -0.6, 'wrist_flex_joint': -1.6511140039870336, 'wrist_roll_joint': -1.5523472976014987})

    def pose_change(self, pose: str="look_with_head"):
        if pose == "look_with_head":
            self.look_with_head()
        elif pose == "look_with_hand":
            self.look_with_hand()
        elif pose == "look_shelf_upper":
            self.look_with_hand(arm_lift_joint=0.1,head_tilt_joint=0.0)
        elif pose == "look_shelf_lower":
            self.look_with_hand(arm_lift_joint=0.0,head_tilt_joint=-0.5)
        elif pose == "look_hand":
            self.look_hand_with_head()
        elif pose == "go_pose":
            self.whole_body.move_to_go()
        elif pose == "neutral_pose":
            self.whole_body.move_to_neutral()
        return

if __name__ == '__main__':
    import rospy
    import hsrb_interface
    import argparse

    parser = argparse.ArgumentParser(description='HSR smach state alone exec')
    parser.add_argument('--pose', '-p', default="look_with_head", help='target pose')
    args = parser.parse_args()
    pose = args.pose

    rospy.init_node('pose_utils_sample')

    print("Initialize robot interface")
    robot = hsrb_interface.Robot()
    from hsrb_interface import settings
    settings._SETTINGS[u'trajectory']['action_timeout'] = 1
    whole_body = robot.try_get('whole_body')
    settings._SETTINGS[u'trajectory']['action_timeout'] = 30.0
    gripper = robot.try_get('gripper')

    act = PoseChange(whole_body, gripper)
    act.pose_change(pose)
