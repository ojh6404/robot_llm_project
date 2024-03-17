#!/usr/bin/env python3

import rospy
import dynamic_reconfigure.client
from llm_common_msgs.msg import Query, Message
from std_srvs.srv import Empty, EmptyResponse
from jsk_recognition_msgs.msg import ClassificationResult
from jsk_recognition_msgs.msg import RectArray

"""
1. ask vlm what object is in interaction
2. ask gd to detect the object
"""


class PerceptionNode(object):
    def __init__(self):
        super(PerceptionNode, self).__init__()
        self.gd_client = dynamic_reconfigure.client.Client("/grounding_dino_node")
        self.vqa_client = dynamic_reconfigure.client.Client("/vqa_node")

        self.sub_cmd = rospy.Subscriber(
            "/llm_agent/perception/query", Query, self.callback, queue_size=1
        )
        self.pub_detection = rospy.Publisher(
            "/llm_agent/perception/detection", Message, queue_size=1
        )

    def query_gd(self, obj_to_query):
        self.gd_config = self.gd_client.get_configuration()
        self.gd_config["classes"] = obj_to_query
        self.gd_client.update_configuration(self.gd_config)
        rospy.sleep(2.0)
        gd_answer = rospy.wait_for_message(
            "/grounding_dino_node/output/class", ClassificationResult
        )
        rect = rospy.wait_for_message("/grounding_dino_node/output/rects", RectArray)
        print(gd_answer)
        print(rect)

    def callback(self, msg):
        vqa_answer = rospy.wait_for_message("/vqa_node/output/text_gen", Message)
        object_detection_query = vqa_answer.message.lower()
        rospy.loginfo("VQA answer: %s", object_detection_query)
        self.gd_config = self.gd_client.get_configuration()
        self.gd_config["classes"] = object_detection_query
        self.gd_client.update_configuration(self.gd_config)
        rospy.sleep(2.0)
        gd_answer = rospy.wait_for_message(
            "/grounding_dino_node/output/class", ClassificationResult
        )
        rect = rospy.wait_for_message("/grounding_dino_node/output/rects", RectArray)
        print(gd_answer)
        print(rect)


if __name__ == "__main__":
    rospy.init_node("perception_node")
    node = PerceptionNode()
    rospy.spin()
