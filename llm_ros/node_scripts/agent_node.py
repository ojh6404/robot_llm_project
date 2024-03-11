#!/usr/bin/env python

"""
ROS node for LLM agent

Subscribes:
    - /llm_agent/task/description (llm_common_msgs/Message)
    - /llm_agent/perception
"""



import rospy
from llm_common.agent import GPTAgent
from llm_common_msgs.msg import Query, Message


class AgentNode(object):
    def __init__(self):
        super(AgentNode, self).__init__()
        self.agent = GPTAgent(model="gpt-4")
        self.sub_task_desc = rospy.Subscriber("/llm_agent/task/description", Message, self.task_callback, queue_size=1)


    def task_callback(self, task_msg):
        print(task_msg.message)
        response_text = self.agent.query(role="user", new_prompt=task_msg.message, messages=[{"role":"system", "content":task_msg.message}])
        test = self.agent.task_parser(response_text)
        print(response_text)
        print(test)



if __name__ == "__main__":
    rospy.init_node("agent_node")
    node = AgentNode()
    rospy.spin()
