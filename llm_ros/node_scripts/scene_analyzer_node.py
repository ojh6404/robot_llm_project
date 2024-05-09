#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np

import torch
import cv2
from PIL import Image as PILImage

import rospy
from sensor_msgs.msg import Image
from llm_common_msgs.msg import ClipResultStamped
from jsk_recognition_msgs.msg import QueryAndProbability
from cv_bridge import CvBridge

from dynamic_reconfigure.server import Server
from llm_ros.cfg import CLIPConfig as ServerConfig


class SceneAnalyzerNode(object):
    def __init__(self):
        super(SceneAnalyzerNode, self).__init__()
        self.bridge = CvBridge()

        self.sub_image = rospy.Subscriber(
            "~input_image",
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24,
        )

        self.pub_debug_image = rospy.Publisher("~debug_image", Image, queue_size=1)
        self.pub_clip_result = rospy.Publisher(
            "~result", ClipResultStamped, queue_size=1
        )

    @torch.no_grad()
    def image_callback(self, image_msg):


if __name__ == "__main__":
    rospy.init_node("scene_analyzer_node")
    node = SceneAnalyzerNode()
    rospy.spin()
