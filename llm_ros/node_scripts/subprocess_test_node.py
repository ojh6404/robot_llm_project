#!/usr/bin/env python

import subprocess
import rospy

import rospkg
import os
import signal

import time


def main():
    rospy.init_node("subprocess_test_node", anonymous=True)
    rospy.loginfo("subprocess_test_node started")
    rospy.loginfo("subprocess_test_node: running subprocess")
    pkg_path = rospkg.RosPack().get_path("tracking_ros")
    cmd = "{}/run_docker -host pr1040 -launch vlpart_segment.launch vocabulary:=custom \"classes:='bottle cap; cup handle;'\"".format(
        pkg_path
    )
    # cmd = "/bin/zsh -c \"source \"/opt/ros/noetic/setup.zsh\" && source \"/home/leus/ros/catkin_ws/devel/setup.zsh\" && rossetip && rossetmaster pr1040 && roslaunch tracking_ros vlpart_segment.launch vocabulary:=custom classes:='bottle cap; cup handle;'\""
    rospy.loginfo("subprocess_test_node: cmd: {}".format(cmd))
    process = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid)

    # kill after 60 seconds
    rospy.loginfo("subprocess_test_node: sleeping for 30 seconds")
    time.sleep(30)
    rospy.loginfo("docker kill")
    subprocess.run(
        'docker kill $(docker ps -aqf "ancestor=tracking_ros:latest")', shell=True
    )
    rospy.loginfo("subprocess_test_node: killing subprocess")
    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
    return_code = process.wait()
    rospy.loginfo("subprocess_test_node: return_code: {}".format(return_code))


if __name__ == "__main__":
    main()
