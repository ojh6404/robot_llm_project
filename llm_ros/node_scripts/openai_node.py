#!/usr/bin/env python

# import tiktoken

import rospy
from sensor_msgs.msg import Image
from llm_common_msgs.srv import Query, QueryResponse
from cv_bridge import CvBridge

from llm_common.agent import GPTAgent

class OpenAINode(object):
    def __init__(self):
        super(OpenAINode, self).__init__()
        self.agent = GPTAgent(model="gpt-4-vision-preview")
        self.bridge = CvBridge()

        # service
        self.query_service = rospy.Service("/test_query", Query, self.query_callback)

    def query_callback(self, req):
        image = self.bridge.imgmsg_to_cv2(req.image, desired_encoding="rgb8")
        query = req.query
        # answer = self.agent.query(role="user", new_prompt=query, messages=[{"role":"system", "content":query}])
        answer = "this is test"
        print("answer: ", answer)
        return QueryResponse(answer=answer)



if __name__ == "__main__":
    rospy.init_node("openai_node")
    node = OpenAINode()
    rospy.spin()
