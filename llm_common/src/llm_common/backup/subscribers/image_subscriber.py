import cv_bridge
import sensor_msgs.msg

from jsk_robocup_common.subscriber import TopicSubscriber


class ImageSubscriber(TopicSubscriber):

    def __init__(self,
                 topic_name,
                 one_shot=False,
                 start=True,
                 wait=False):
        super(ImageSubscriber, self).__init__(
            topic_name,
            sensor_msgs.msg.Image,
            one_shot=one_shot,
            start=start,
            wait=wait)
        self.bridge = cv_bridge.CvBridge()

    @property
    def image(self):
        if self.msg is None:
            return None
        cv_image = self.bridge.imgmsg_to_cv2(self.msg, 'bgr8')
        return cv_image
