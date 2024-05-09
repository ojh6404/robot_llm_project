#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import pandas as pd
import rospy

from llm_common_msgs.msg import Float32MultiArrayStamped


class DataFilterNode(object):
    def __init__(self):
        self.window_size = window_size if window_size % 2 == 1 else window_size + 1
        self.poly_order = poly_order
        self.data_queue = []
        self.pub_filtered_data = rospy.Publisher('~filtered', Float32, queue_size=1)
        self.sub_raw_data = rospy.Subscriber('~raw', Float32MultiArrayStamped, self.cb_raw_data, queue_size=1)
