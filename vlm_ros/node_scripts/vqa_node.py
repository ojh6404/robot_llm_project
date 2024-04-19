#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import json
import base64
import requests
from requests.exceptions import ConnectionError

import rospy
from dynamic_reconfigure.server import Server
from vlm_ros.cfg import VLMConfig as ServerConfig
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from llm_common_msgs.msg import (
    Float32MultiArrayStamped,
    Int64MultiArrayStamped,
    GenerationOutput,
)
from sensor_msgs.msg import Image


# query node to request via flask
class QueryNode(object):
    def __init__(self):
        self.host = rospy.get_param("~host", "localhost")
        self.port = rospy.get_param("~port", 8888)
        self.app_name = rospy.get_param("~task_name", "text_gen")
        self.output_logits = rospy.get_param("~output_logits", False)
        self.gen_config = dict()  # placeholder
        self.reconfigure_server = Server(ServerConfig, self.config_cb)

        self.bridge = CvBridge()
        self.pub_text = rospy.Publisher(
            f"~output/{self.app_name}", String, queue_size=1
        )
        self.pub_gen_output = rospy.Publisher(
            f"~output/generation_output", GenerationOutput, queue_size=1
        )
        self.sub_img = rospy.Subscriber(
            "~input_image", Image, self.callback, queue_size=1, buff_size=2**24
        )
        self.pub_img = rospy.Publisher("~output/debug_image", Image, queue_size=1)

    def config_cb(self, config, level):
        self.queries = [query.strip() for query in config.queries.split(";")]
        self.gen_config["do_sample"] = config.do_sample
        self.gen_config["top_k"] = config.top_k
        self.gen_config["max_length"] = config.max_length
        self.gen_config["temperature"] = config.temperature
        rospy.loginfo("Reconfigure Request: {}".format(config))
        return config

    def callback(self, msg):
        # query flask server for infer result when image received
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        else:
            # TODO : batch
            query = self.queries[0].replace("\\n", "\n")
            result = self.inference(img, [query])
            text_msg = String()
            generation_output_msg = GenerationOutput(header=msg.header)
            info_log = ""
            answer = result["answers"][0]
            if "top_p_output_logits" in result:
                logits = result["top_p_output_logits"]["logits"]  # (text_length, top_p)
                token_ids = result["top_p_output_logits"][
                    "token_ids"
                ]  # (text_length, top_p)
                info_log += "logits : {}\n".format(logits)
                info_log += "token_ids : {}\n".format(token_ids)
                if self.output_logits:
                    logits_data = []
                    tokend_ids_data = []
                    for logit, token_id in zip(logits, token_ids):
                        logit_msg = Float32MultiArrayStamped(data=logit)
                        token_id_msg = Int64MultiArrayStamped(data=token_id)
                        logits_data.append(logit_msg)
                        tokend_ids_data.append(token_id_msg)
                    generation_output_msg.logits = logits_data
                    generation_output_msg.token_ids = tokend_ids_data
            rospy.loginfo(info_log + "answer : {}".format(answer))
            text_msg.data = answer
            generation_output_msg.gen_output = answer
            self.pub_text.publish(text_msg)
            self.pub_gen_output.publish(generation_output_msg)

            # put text on debug image
            cv2.putText(
                img,
                "question : " + query,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                img,
                "answer : " + answer,
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            img_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
            self.pub_img.publish(img_msg)

    def send_request(self, content, headers=None):
        url = "http://{}:{}/{}".format(self.host, self.port, self.app_name)
        try:
            response = requests.post(url, data=content, headers=headers)
        except ConnectionError as e:
            rospy.logwarn_once(
                "Cannot establish the connection with API server. Is it running?"
            )
            raise e
        else:
            if response.status_code == 200:
                return response
            else:
                err_msg = "Invalid http status code: {}".format(
                    str(response.status_code)
                )
                rospy.logerr(err_msg)
                raise RuntimeError(err_msg)

    def cv_img_to_byte(self, img):
        _, encimg = cv2.imencode(".png", img)
        img_byte = base64.b64encode(encimg).decode("utf-8")  # type: ignore[arg-type]
        return img_byte

    def inference(self, img, queries):
        img_byte = self.cv_img_to_byte(img)
        headers = {"Content-Type": "application/json"}
        req = json.dumps(
            {"image": img_byte, "queries": queries, "gen_config": self.gen_config}
        )
        response = self.send_request(req, headers=headers)
        result = json.loads(response.text)
        return result


if __name__ == "__main__":
    rospy.init_node("query_node")
    query_node = QueryNode()
    rospy.spin()
