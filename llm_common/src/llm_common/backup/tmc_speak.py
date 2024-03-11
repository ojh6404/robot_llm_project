#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# c.f. The original program is https://github.com/jsk-ros-pkg/jsk_hsr/blob/master/jsk_hsr_startup/src/jsk_hsr_startup/tmc_speak.py

import actionlib
import actionlib_msgs.msg
import rospy
from tmc_msgs.msg import TalkRequestAction
from tmc_msgs.msg import TalkRequestGoal
import tmc_msgs.msg


_sound_play_clients = {}


def play_sound(
    sentence,
    lang=tmc_msgs.msg.Voice.kJapanese,
    topic_name="/talk_request_action",
    wait=False,
):
    """Plays sound using sound_play server for TMC

    Parameters
    ----------
    sentence : string
        words spoken by robot.
    lang : int
        tmc_msgs.msg.Voice.kEnglish or tmc_msgs.msg.Voice.kJapanese
    topic_name : string
        topic name: default is 'talk_request_action'.
    wait : bool
        if the wait is True, wait until speaking ends.

    Returns
    -------
    client : actionlib.SimpleActionClient
        return SimpleActionClient

    """
    if lang not in [tmc_msgs.msg.Voice.kEnglish, tmc_msgs.msg.Voice.kJapanese]:
        raise ValueError("lang is invalid.")

    if topic_name in _sound_play_clients:
        client = _sound_play_clients[topic_name]
    else:
        client = actionlib.SimpleActionClient(topic_name, TalkRequestAction)
    client.wait_for_server()

    goal = TalkRequestGoal()
    if client.get_state() == actionlib_msgs.msg.GoalStatus.ACTIVE:
        client.cancel_goal()
        client.wait_for_result(timeout=rospy.Duration(10))
    goal.data.sentence = sentence
    goal.data.language = lang
    _sound_play_clients[topic_name] = client
    client.send_goal(goal)

    if wait is True:
        client.wait_for_result(timeout=rospy.Duration(10))
    return client


def speak_en(sentence, topic_name="/talk_request_action", wait=False):
    """Speak english sentence

    Parameters
    ----------
    sentence : string
        words spoken by robot.
    topic_name : string
        topic name: default is 'talk_request_action'.
    wait : bool
        if the wait is True, wait until speaking ends.

    Returns
    -------
    client : actionlib.SimpleActionClient
        return SimpleActionClient

    Examples
    --------
    >>> import rospy
    >>> rospy.init_node("tmc_speak_example")
    >>> from jsk_robocup_common.tmc_speak import speak_en
    >>> speak_en('This is a test sentence', wait=True)
    <actionlib.simple_action_client.SimpleActionClient instance at 0x7fd940979c20>
    >>> speak_en('This is a test sentence', wait=False)
    <actionlib.simple_action_client.SimpleActionClient instance at 0x7fd940979c20>
    """
    return play_sound(
        sentence, lang=tmc_msgs.msg.Voice.kEnglish, topic_name=topic_name, wait=wait
    )


def speak_jp(sentence, topic_name="/talk_request_action", wait=False):
    """Speak japanese sentence

    Parameters
    ----------
    sentence : string
        words spoken by robot.
    topic_name : string
        topic name: default is 'talk_request_action'.
    wait : bool
        if the wait is True, wait until speaking ends.

    Returns
    -------
    client : actionlib.SimpleActionClient
        return SimpleActionClient

    Examples
    --------
    >>> import rospy
    >>> rospy.init_node("tmc_speak_example")
    >>> from jsk_robocup_common.tmc_speak import speak_jp
    >>> speak_jp('テストです')
    <actionlib.simple_action_client.SimpleActionClient instance at 0x7fd940979c20>
    >>> speak_jp('テストです', wait=True)
    <actionlib.simple_action_client.SimpleActionClient instance at 0x7fd940979c20>
    """
    return play_sound(
        sentence, lang=tmc_msgs.msg.Voice.kJapanese, topic_name=topic_name, wait=wait
    )


def speak_jp_en(
    jp_sentence, en_sentence, topic_name="/talk_request_action", wait=False
):
    """Speak japanese or english sentence

    This function reads rosparam 'speak_language' and speak
    japanese or english sentence.
    If value of 'speak_language' is not set, speak in japanese.

    Parameters
    ----------
    jp_sentence : string
        words spoken by robot in japanese.
    en_sentence : string
        words spoken by robot in english.
    topic_name : string
        topic name: default is 'talk_request_action'.
    wait : bool
        if the wait is True, wait until speaking ends.

    Returns
    -------
    client : actionlib.SimpleActionClient
        return SimpleActionClient

    Examples
    --------
    >>> import rospy
    >>> rospy.init_node("tmc_speak_example")
    >>> from jsk_robocup_common.tmc_speak import speak_jp_en
    >>> rospy.set_param('speak_language', 'jp')
    >>> speak_jp_en('テストです', 'This is a test sentence.')
    >>> rospy.set_param('speak_language', 'en')
    >>> speak_jp_en('テストです', 'This is a test sentence.')
    """
    speak_language = rospy.get_param("speak_language", None)
    if speak_language == "en":
        speak_function = speak_en
        sentence = en_sentence
    else:
        speak_function = speak_jp
        sentence = jp_sentence
    return speak_function(sentence, topic_name=topic_name, wait=wait)
