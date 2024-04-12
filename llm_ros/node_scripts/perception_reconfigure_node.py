#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import dynamic_reconfigure.client
from llm_common_msgs.srv import PerceptionReconfigure, PerceptionReconfigureResponse


class PerceptionReconfigureNode(object):
    def __init__(self):
        super(PerceptionReconfigureNode, self).__init__()
        self.dino_node = rospy.get_param("~dino_node", "/grounding_dino_node")
        self.yolo_node = rospy.get_param("~yolo_node", "/yolo_node")
        self.vlpart_node = rospy.get_param("~vlpart_node", "/vlpart_reconfigure_node")
        self.clip_node = rospy.get_param("~clip_node", "/clip_inference_node")
        self.vqa_node = rospy.get_param("~vqa_node", "/vqa_node")

        self.perception_query_service = rospy.Service(
            "/perception_reconfigure_node/reconfigure", PerceptionReconfigure, self.perception_reconfigure_service_cb
        )

    def perception_reconfigure_service_cb(self, req):
        detector = req.detector
        detection_query = req.query
        rospy.loginfo("Reconfiguring {} with query: {}".format(detector, detection_query))
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
        elif detector == "clip":
            self.clip_client = dynamic_reconfigure.client.Client(self.clip_node)
            self.clip_client.update_configuration({"description": detection_query})
        else:
            rospy.logerr("Invalid detector: {}".format(detector))
            return PerceptionReconfigureResponse(result=False)
        return PerceptionReconfigureResponse(result=True)


if __name__ == "__main__":
    rospy.init_node("perception_reconfigure_node")
    rospy.loginfo("Starting perception reconfigure node")
    node = PerceptionReconfigureNode()
    rospy.spin()
