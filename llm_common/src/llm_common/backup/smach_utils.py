#!/usr/bin/env python3
import rospy
import hashlib


def gen_smach_label(command_name: str, arg: str, hashvalue: str) -> str:
    if command_name == "end":
        return "end"
    else:
        return "{}-{}-{}".format(command_name, arg, hashvalue)


def gen_hash(length: int = 6) -> str:
    time_s = str(rospy.Time.now().secs) + str(rospy.Time.now().nsecs)
    return hashlib.sha256(time_s.encode("utf-8")).hexdigest()[:length]
