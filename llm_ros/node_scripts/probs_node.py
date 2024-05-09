#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np

import rospy
from sensor_msgs.msg import Image
# from llm_common_msgs.msg import ClipResultStamped
# from jsk_recognition_msgs.msg import QueryAndProbability
from llm_common_msgs.msg import Float32MultiArrayStamped, GenerationOutput

class ProbsNode(object):
    def __init__(self):
        super(ProbsNode, self).__init__()
        self.sub_gen_output = rospy.Subscriber(
            "~gen_output",
            GenerationOutput,
            self.gen_output_callback,
            queue_size=1,
            buff_size=2**24,
        )

        self.pub_gen_probs = rospy.Publisher(
            "~gen_probs",
            Float32MultiArrayStamped,
            queue_size=1,
        )

        # TODO add ClipResult

    def gen_output_callback(self, gen_output_msg):
        text = gen_output_msg.gen_output # text should include "Yes" or "No
        token_ids = gen_output_msg.token_ids[0].data[:2]
        # 1939: No, 3869: Yes
        assert set(token_ids) == set([1939, 3869]), f"Invalid token_ids: {token_ids}"
        # assert text.lower() in ["yes", "no", "yes.", "no."], f"Invalid text: {text}"
        logits = np.array(gen_output_msg.logits[0].data)
        logits -= np.max(logits)
        # calculate probabilities from logits
        exp_logits = np.exp(logits)
        probs = exp_logits / np.sum(exp_logits)
        probs = probs.tolist()
        # publish probabilities
        gen_probs_msg = Float32MultiArrayStamped(header=gen_output_msg.header)
        if text.lower() == "no" or text.lower() == "no.":
            gen_probs_msg.data = [1 - probs[0], probs[0]]
        else:
            gen_probs_msg.data = probs
        self.pub_gen_probs.publish(gen_probs_msg)



if __name__ == "__main__":
    rospy.init_node("probs_node")
    node = ProbsNode()
    rospy.loginfo("probs_node started")
    rospy.spin()
