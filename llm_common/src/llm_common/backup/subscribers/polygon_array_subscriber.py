import jsk_recognition_msgs.msg

from jsk_robocup_common.subscriber import TopicSubscriber
from jsk_robocup_common.models.polygon import Polygon


class PolygonArraySubscriber(TopicSubscriber):

    def __init__(self, topic_name, one_shot=False, start=True, wait=False):
        super(PolygonArraySubscriber, self).__init__(
            topic_name,
            jsk_recognition_msgs.msg.PolygonArray,
            one_shot=one_shot,
            start=start,
            wait=wait,
        )

    def to_coords(self, msg=None):
        if msg is None:
            msg = self.msg
        return [
            Polygon(points=[(p.x, p.y, p.z) for p in poly.polygon.points])
            for poly in msg.polygons
        ]
