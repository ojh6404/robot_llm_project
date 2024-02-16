from enum import IntEnum

import jsk_recognition_msgs.msg
import numpy as np
import skrobot

from jsk_robocup_common.subscriber import TopicSubscriber
from jsk_robocup_common.models.box import BoundingBox


class BoundingBoxArraySubscriber(TopicSubscriber):

    def __init__(self,
                 topic_name,
                 one_shot=False,
                 start=True,
                 wait=False):
        super(BoundingBoxArraySubscriber, self).__init__(
            topic_name,
            jsk_recognition_msgs.msg.BoundingBoxArray,
            one_shot=one_shot,
            start=start,
            wait=wait)

    def to_coords(self):
        return [BoundingBox.from_ros_message(box)
                for box in self.msg.boxes]
