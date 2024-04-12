#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import dynamic_reconfigure.client
from llm_common_msgs.srv import PerceptionQuery, PerceptionQueryResponse


class PerceptionNode(object):
    def __init__(self):
        super(PerceptionNode, self).__init__()
        self.dino_node = rospy.get_param("~dino_node", "/grounding_dino_node")
        self.yolo_node = rospy.get_param("~yolo_node", "/yolo_node")
        self.vlpart_node = rospy.get_param("~vlpart_node", "/vlpart_reconfigure_node")
        self.vqa_node = rospy.get_param("~vqa_node", "/vqa_node")

        self.perception_query_service = rospy.Service(
            "/perception_node/query", PerceptionQuery, self.perception_query_service_cb
        )

    def perception_query_service_cb(self, req):
        detector = req.detector
        detection_query = req.query
        if detector == "grounding_dino":
            self.gd_client = dynamic_reconfigure.client.Client(self.dino_node)
            self.gd_client.update_configuration({"classes": detection_query})
        elif detector == "yolo":
            self.yolo_client = dynamic_reconfigure.client.Client(self.yolo_node)
            self.yolo_client.update_configuration({"classes": detection_query})
        elif detector == "vlpart":
            self.vlpart_client = dynamic_reconfigure.client.Client(self.vlpart_node)
            self.vlpart_client.update_configuration({"classes": detection_query})
        elif detector == "vqa":
            self.vqa_client = dynamic_reconfigure.client.Client(self.vqa_node)
            self.vqa_client.update_configuration({"question": detection_query})
        else:
            rospy.logerr("Invalid detector: {}".format(detector))
            return PerceptionQueryResponse(result=False)
        return PerceptionQueryResponse(result=True)


if __name__ == "__main__":
    rospy.init_node("perception_node")
    rospy.loginfo("Starting perception node")
    node = PerceptionNode()
    rospy.spin()
