#!/usr/bin/env python

import numpy as np

import rospy
import dynamic_reconfigure.client
from sensor_msgs.msg import Joy
from hand_object_detection_ros.msg import HandDetectionArray
from llm_common_msgs.msg import ClipResultStamped
from cv_bridge import CvBridge

from eus_imitation.cfg import GroundingDINOConfig as ServerConfig
from message_filters import ApproximateTimeSynchronizer, Subscriber

def callback(config):
    print("this is config")
    print(config)

class HandJoyNode(object):
    def __init__(self):
        super(HandJoyNode, self).__init__()
        self.reconfigure_server = Server(ServerConfig, self.config_cb)
        self.client = dynamic_reconfigure.client.Client("~clip_node", timeout=30, config_callback=callback)
        self.client.update_configuration({"description": "a man holding a cup; a man doing nothing; a man not holding a cup;"})

        self.clip_threshold = 0.5
        self.hand_dist_threshold = 0.1

        self.pub_joy = rospy.Publisher("/eus_imitation/hand/joy", Joy, queue_size=1)

        self.sub_hand_detections = Subscriber("/hand_object_detection_ros/hand_detections", HandDetectionArray)
        self.sub_clip_result = Subscriber("/eus_imitation/clip_result", ClipResultStamped)

        self.ts = ApproximateTimeSynchronizer([self.sub_hand_detections, self.sub_clip_result], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.hand_detection_callback)


    def hand_detection_callback(self, hand_detections, clip_result):
        rospy.loginfo("hand detection callback")
        rospy.loginfo("hand_detections: %s", hand_detections)
        rospy.loginfo("clip_result: %s", clip_result)

        joy = Joy()
        joy.header.stamp = rospy.Time.now()
        joy.header.frame_id = "hand_joy"
        joy.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        joy.buttons = [0, 0, 0, 0, 0, 0, 0, 0]

        self.pub_joy.publish(joy)



if __name__ == "__main__":
    rospy.init_node("hand_joy_node")
    policy_running_node = HandJoyNode()
    rospy.spin()
