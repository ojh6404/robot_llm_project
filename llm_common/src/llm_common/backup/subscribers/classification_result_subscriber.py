import jsk_recognition_msgs.msg
import rospy

from jsk_robocup_common.subscriber import TopicSubscriber


class ClassificationResultSubscriber(TopicSubscriber):

    def __init__(self, topic_name):
        super(ClassificationResultSubscriber, self).__init__(
            topic_name,
            jsk_recognition_msgs.msg.ClassificationResult)

    @property
    def max_label(self):
        if self.msg is None:
            return None
        return self.msg.label_names[0]

    def wait_until_score(self, label_name, threshold=0.1):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
            target_score = 'none'
            if self.msg is not None:
                index = self.msg.label_names.index(label_name)
                if index == -1:
                    rospy.logwarn('[ClassificationResultSubscriber] could not find target_label {}'.format(label_name))
                    continue
                target_score = self.msg.label_proba[index]
            if target_score > threshold:
                break
            rospy.loginfo('[ClassificationResultSubscriber] waiting {} {} > {}'.format(
                label_name,
                target_score,
                threshold))
