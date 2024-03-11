import geometry_msgs.msg

from jsk_robocup_common.subscriber import TopicSubscriber


class WrenchStampedSubscriber(TopicSubscriber):

    def __init__(self, topic_name="~input", one_shot=False, start=True, wait=False):
        super(WrenchStampedSubscriber, self).__init__(
            topic_name,
            geometry_msgs.msg.WrenchStamped,
            one_shot=one_shot,
            start=start,
            wait=wait,
        )
