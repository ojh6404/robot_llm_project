import geometry_msgs.msg
from skrobot.coordinates import Coordinates
from skrobot.interfaces.ros.tf_utils import geometry_pose_to_coords

from jsk_robocup_common.subscriber import TopicSubscriber


class PoseArraySubscriber(TopicSubscriber):

    def __init__(self, topic_name, one_shot=False, start=True, wait=False):
        super(PoseArraySubscriber, self).__init__(
            topic_name,
            geometry_msgs.msg.PoseArray,
            one_shot=one_shot,
            start=start,
            wait=wait,
        )

    def to_coords(self):
        return [geometry_pose_to_coords(pose) for pose in self.msg.poses]
