#!/usr/bin/env python

import numpy as np

import rospy
import dynamic_reconfigure.client
from sensor_msgs.msg import Joy
from hand_object_detection_ros.msg import HandDetectionArray
from llm_common_msgs.msg import ClipResultStamped
from cv_bridge import CvBridge

from dynamic_reconfigure.server import Server
from llm_ros.cfg import HandDetectionConfig as ServerConfig
from message_filters import ApproximateTimeSynchronizer, Subscriber

def callback(config):
    rospy.loginfo("Reconfigure Request: %s", config)

class HandJoyNode(object):
    def __init__(self):
        super(HandJoyNode, self).__init__()
        self.client = dynamic_reconfigure.client.Client("/clip_interaction_detector", timeout=30, config_callback=callback)
        self.reconfigure_server = Server(ServerConfig, self.config_cb)

        self.pub_left_joy = rospy.Publisher("~left_hand/joy", Joy, queue_size=1)
        self.pub_right_joy = rospy.Publisher("~right_hand/joy", Joy, queue_size=1)

        self.debug = rospy.get_param("~debug", False)

        if self.debug:
            from std_msgs.msg import Float32MultiArray
            self.pub_dist = rospy.Publisher("~hand_distance", Float32MultiArray, queue_size=1)

        self.clip_threshold = rospy.get_param("~clip_threshold", 0.8)
        self.hand_dist_threshold = rospy.get_param("~hand_dist_threshold", 0.09)

        self.sub_hand_detections = Subscriber("~hand_detections", HandDetectionArray)
        self.sub_clip_result = Subscriber("~clip_result", ClipResultStamped)

        self.ts = ApproximateTimeSynchronizer([self.sub_hand_detections, self.sub_clip_result], queue_size=1, slop=0.15)
        self.ts.registerCallback(self.hand_detection_callback)


    def config_cb(self, config, level):
        self.clip_threshold = config["clip_threshold"]
        self.hand_dist_threshold = config["hand_dist_threshold"]
        self.objects = [_obj.strip() for _obj in config.objects.split(";") if _obj.strip()]
        self.description = "; ".join(["a man holding a {}".format(obj) for obj in self.objects] + ["a man doing nothing;"])
        self.client.update_configuration({"description": self.description})
        return config

    def hand_detection_callback(self, hand_detections, clip_result):
        probs = [res.probability for res in clip_result.clip.result] # TODO use clip result for detecting hand-object interaction
        for hand_detection in hand_detections.detections:
            joy = Joy(header=hand_detections.header)
            joy.axes = [0, 0, 0, 0, 0, 0] # does not need

            thumb_coords = [hand_detection.skeleton.bones[3].end_point.x, hand_detection.skeleton.bones[3].end_point.y, hand_detection.skeleton.bones[3].end_point.z]
            index_coords = [hand_detection.skeleton.bones[7].end_point.x, hand_detection.skeleton.bones[7].end_point.y, hand_detection.skeleton.bones[7].end_point.z]
            dist = np.linalg.norm(np.array(thumb_coords) - np.array(index_coords))
            if dist < self.hand_dist_threshold:
                joy.buttons = [1] # grasp
            else:
                joy.buttons = [0] # release
            if hand_detection.hand == "left_hand":
                self.pub_left_joy.publish(joy)
            elif hand_detection.hand == "right_hand":
                self.pub_right_joy.publish(joy)
                if self.debug:
                    self.pub_dist.publish(Float32MultiArray(data=[dist]))


if __name__ == "__main__":
    rospy.init_node("hand_joy_node")
    policy_running_node = HandJoyNode()
    rospy.spin()
