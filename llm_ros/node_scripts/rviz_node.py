#!/usr/bin/env python

import math
import rospy

from std_msgs.msg import Float32
from llm_common_msgs.msg import ClipResultStamped


class RvizNode(object):
    def __init__(self):
        super(RvizNode, self).__init__()

        self.sub_clip = rospy.Subscriber(
            '/clip_inference_node/result', ClipResultStamped, self.clip_cb)


        self.pubs = [rospy.Publisher(
            '/clip_prob' + str(i), Float32, queue_size=1) for i in range(0, 4)]

    def clip_cb(self, msg):
        clip_result = msg.clip.result
        for i in range(len(clip_result)):
            self.pubs[i].publish(clip_result[i].probability)




if __name__ == '__main__':
    rospy.init_node('rviz_node')
    node = RvizNode()
    rospy.spin()
