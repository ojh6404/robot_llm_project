#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from llm_common_msgs.srv import Query, QueryRequest

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    # test call rosservice
    image_msg = rospy.wait_for_message("/kinect_head/rgb/image_rect_color", Image)
    rospy.wait_for_service("/llm_ros/openai_node/query")
    query_service = rospy.ServiceProxy("/llm_ros/openai_node/query", Query)
    query = QueryRequest()
    query.query = "describe the image."
    query.image = image_msg


    response = query_service(query)
    print(response.answer)


if __name__ == "__main__":
    rospy.init_node("test_node")
    main()
    rospy.spin()
