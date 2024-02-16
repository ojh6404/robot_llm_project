from collections import namedtuple

import threading
import cv_bridge
import sensor_msgs.msg
import message_filters
import jsk_robocup_common_msgs.msg


ImgWithFeature = namedtuple('ImgWithFeature', ('img', 'feature'))


class RectWithFeatureArraySubscriber(object):

    def __init__(self,
                 image_topic_name,
                 rect_array,
                 approximate_sync=False,
                 queue_size=30,
                 slop=0.1):
        super(RectWithFeatureArraySubscriber, self).__init__()
        self.queue_size = queue_size
        self.approximate_sync = approximate_sync
        self.slop = slop

        self.lock = threading.Lock()
        self.bridge = cv_bridge.CvBridge()
        self.image_topic_name = image_topic_name
        self.rect_array = rect_array
        self.subscribe()

    def subscribe(self):
        self.img = None
        queue_size = self.queue_size
        sub_img = message_filters.Subscriber(
            self.image_topic_name,
            sensor_msgs.msg.Image,
            queue_size=1)
        sub_rect_array = message_filters.Subscriber(
            self.rect_array,
            jsk_robocup_common_msgs.msg.RectWithFeatureArray,
            queue_size=1)
        self.subs = [sub_img, sub_rect_array]
        if self.approximate_sync:
            slop = self.slop
            sync = message_filters.ApproximateTimeSynchronizer(
                fs=self.subs, queue_size=queue_size, slop=slop)
        else:
            sync = message_filters.TimeSynchronizer(
                fs=self.subs, queue_size=queue_size)
        sync.registerCallback(self._cb)

    def unsubscribe(self):
        for s in self.subs:
            s.unregister()

    def _cb(self, img_msg, rect_msg):
        img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        self.lock.acquire()
        self.img = img
        self.rect_msg = rect_msg
        self.lock.release()

    def get_crop_imgs_with_feature(self, discard=True):
        if self.img is None:
            return []
        imgs = []
        self.lock.acquire()
        for rect in self.rect_msg.rects:
            x_min = rect.x
            y_min = rect.y
            x_max = rect.x + rect.width + 1
            y_max = rect.y + rect.height + 1
            imgs.append(
                ImgWithFeature(self.img[y_min:y_max, x_min:x_max].copy(),
                               rect.features))
        self.lock.release()
        if discard:
            self.img = None
            self.rect_msg = None
        return imgs


if __name__ == '__main__':
    import rospy
    rospy.init_node('rect_with_feature_array_subscriber_test')
    sub = RectWithFeatureArraySubscriber(
        '/remote/qhd/image_color_rect',
        '/interact/label_filter/arcface_classification/output/rects_with_feature')
