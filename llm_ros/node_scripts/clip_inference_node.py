#!/usr/bin/env python

import clip
import torch
import cv2
from PIL import Image as PILImage

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

from dynamic_reconfigure.server import Server
from llm_ros.cfg import CLIPConfig as ServerConfig


class CLIPInferenceNode(object):
    def __init__(self):
        super(CLIPInferenceNode, self).__init__()
        self.reconfigure_server = Server(ServerConfig, self.config_cb)
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model, self.preprocess = clip.load("ViT-L/14@336px", device=self.device)
        self.bridge = CvBridge()

        self.sub_image = rospy.Subscriber(
            "~input_image", Image, self.image_callback, queue_size=1
        )
        self.pub_similarity = rospy.Publisher(
            "/clip/similarity", Float32MultiArray, queue_size=1
        )
        self.pub_image_with_similarity = rospy.Publisher(
            "/clip/debug_image", Image, queue_size=1
        )

    def config_cb(self, config, level):
        self.desc = [_desc.strip() for _desc in config.description.split(";") if _desc.strip()]
        rospy.loginfo(f"Descriptions: {self.desc}")
        return config

    def image_callback(self, image_msg):
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="rgb8")
        image = PILImage.fromarray(cv_image)

        image_input = self.preprocess(image).unsqueeze(0).to(self.device)
        text_input = clip.tokenize(self.desc).to(self.device)

        with torch.no_grad():
            image_features = self.model.encode_image(image_input)
            text_features = self.model.encode_text(text_input)

            image_features /= image_features.norm(dim=-1, keepdim=True)
            text_features /= text_features.norm(dim=-1, keepdim=True)

            similarity = (100.0 * image_features @ text_features.T).softmax(dim=-1)

        similarity_msg = Float32MultiArray()
        similarity_msg.data = [sim.item() for sim in similarity[0]]
        self.pub_similarity.publish(similarity_msg)

        # draw text on the image with cv2
        for i, desc in enumerate(self.desc):
            cv_image = cv2.putText(
                cv_image,
                f"{desc}: {similarity[0][i].item():.2f}",
                (10, 30 * (i + 1)),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        image_with_similarity_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="rgb8")
        self.pub_image_with_similarity.publish(image_with_similarity_msg)


if __name__ == "__main__":
    rospy.init_node("clip_inference_node")
    node = CLIPInferenceNode()
    rospy.spin()
