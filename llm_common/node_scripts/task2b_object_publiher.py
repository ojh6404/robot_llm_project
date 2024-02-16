#!/usr/bin/env python3

import os
import rospy
import roslib.packages
from std_msgs.msg import String

class ObjectNamePublisher(object):
    def __init__(self):
        config_name = "tidy_up_task_2b_obejct.txt"
        config_path = os.path.join(roslib.packages.get_pkg_dir("jsk_robocup_demo"), "cfg", config_name)
        self.object_name_file = config_path
        self.object_name = ""
        try:
            with open(self.object_name_file, "r") as f:
                self.object_name = f.read()
        except FileNotFoundError:
            pass

        self.pub = rospy.Publisher('task_2b_object_name', String, queue_size=10)
        self.sub = rospy.Subscriber('ordered_object_name', String, self.string_cb)
        duration = rospy.Duration(0.1)
        self.timer = rospy.Timer(duration, self.timer_cb)
        rospy.loginfo("ObjectName Publisher Server initialized")

    def string_cb(self, data: String):
        rospy.loginfo("Received object name: %s", data.data)
        self.object_name = data.data
        with open(self.object_name_file, "w") as f:
            f.write(data.data)

    def timer_cb(self, event):
        self.pub.publish(self.object_name)

if __name__ == "__main__":
    rospy.init_node('object_name_to_topic_publisher', anonymous=True)
    server = ObjectNamePublisher()
    rospy.spin()
