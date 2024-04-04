#!/usr/bin/env python

import os
from openai import OpenAI
import base64
import cv2
import numpy as np

import rospy
from sensor_msgs.msg import Image
from llm_common_msgs.srv import Query, QueryResponse
from cv_bridge import CvBridge

from llm_common.prompts import SYSTEM_PROMPT, PRIMITIVES

def cv2_to_base64(cv2_image: np.ndarray) -> str:
    """
    encode cv2 image to base64
    """
    base64_image = base64.b64encode(cv2.imencode(".jpg", cv2_image)[1]).decode("utf-8")
    return base64_image

def create_message(role: str, text: str, cv2_image: np.ndarray = None) -> dict:
    """
    create a message for the GPT-4 agent
    role : "user" or "system" # str
    text : the text of the message # str
    image : the image of the message # np.ndarray [H, W, 3]
    """
    assert role in ["user", "system"]

    if cv2_image is None:
        content = {"role": role, "content": [{"type": "text", "text": text}]}
    else:
        base64_image = cv2_to_base64(cv2_image=cv2_image)
        content = {
            "role": role,
            "content": [
                {"type": "text", "text": text},
                {
                    "type": "image_url",
                    "image": f"data:image/jpeg;base64,{base64_image}",
                },
            ],
        }
    return content

class OpenAINode(object):
    def __init__(self):
        super(OpenAINode, self).__init__()
        self.bridge = CvBridge()

        # openai client
        self.model = rospy.get_param("~model", "gpt-4-vision-preview")
        self.client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
        self.stop = None
        self.seed = 123
        self.logprobs = True
        self.top_logprobs = 2

        self.messages = []
        # self.messages.append({"role": "system", "content": SYSTEM_PROMPT + PRIMITIVES})
        self.max_token = 4096
        self.max_completion_length = 2000
        self.last_response = None

        self.payload = {
            "model": self.model,
            "messages": [],  # ここに質問や画像などを並べてモデルに投げる。
            "max_tokens": self.max_token,
        }

        # service
        self.query_service = rospy.Service(
            "/llm_ros/openai_node/query", Query, self.query_callback
        )

    def query_callback(self, req):
        query = req.query
        cv2_image = (
            self.bridge.imgmsg_to_cv2(req.image, desired_encoding="bgr8")
            if req.image
            else None
        )
        try:
            # self.add_message_entry_as_specified_role(role="user")
            self.add_message_entry_as_specified_role(role="user")
            self.add_text_content(text="What do you see in this image?")
            b64image = self.cv2_to_base64(cv2_image=cv2_image)
            self.add_b64image_content(b64image=b64image)
            answer = self.execute()
            # answer = response.choices[0].message.content
            return QueryResponse(answer=answer)
        except Exception as e:
            print(f"An error occurred: {e}")
            return None


if __name__ == "__main__":
    rospy.init_node("openai_node")
    node = OpenAINode()
    rospy.spin()
