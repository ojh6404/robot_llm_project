import datetime

import rospy
from speech_recognition_msgs.msg import SpeechRecognitionCandidates

from jsk_robocup_common.subscriber import TopicSubscriber


class SpeechRecognitionSubscriber(TopicSubscriber):

    def __init__(
        self, topic_name="/Tablet/voice", one_shot=False, start=True, wait=False
    ):
        super(SpeechRecognitionSubscriber, self).__init__(
            topic_name,
            SpeechRecognitionCandidates,
            one_shot=one_shot,
            start=start,
            wait=wait,
        )

    def wait_specific_words(self, words):
        rate = rospy.Rate(10)
        start = datetime.datetime.now()
        cur_start = start
        self.msg = None
        while not rospy.is_shutdown():
            if self.msg is not None:
                voice_data = self.msg.transcript[0]
                if voice_data in words:
                    break
            rate.sleep()
            cur_time = datetime.datetime.now()
            dt = cur_time - cur_start
            if dt > datetime.timedelta(seconds=self.warn_timeout):
                dt = cur_time - start
                rospy.logwarn(
                    "Topic {} not received for {} seconds".format(self.topic_name, dt)
                )
                cur_start = datetime.datetime.now()
