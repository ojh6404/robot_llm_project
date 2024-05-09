#!/usr/bin/env python
#-*- coding: utf-8 -*-

import torch
from torchvision import transforms
import cv2
from PIL import Image as PILImage

import rospy
from sensor_msgs.msg import Image
from llm_common_msgs.msg import ClipResultStamped
from jsk_recognition_msgs.msg import QueryAndProbability
from cv_bridge import CvBridge

from dynamic_reconfigure.server import Server
from llm_ros.cfg import CLIPConfig as ServerConfig

from imagebind.models import imagebind_model
from imagebind.models.imagebind_model import ModalityType
from imagebind.models.multimodal_preprocessors import SimpleTokenizer

import rospkg
BPE_PATH = rospkg.RosPack().get_path("llm_ros") + "/bpe/bpe_simple_vocab_16e6.txt.gz"

def load_and_transform_text(text, device):
    if text is None:
        return None
    tokenizer = SimpleTokenizer(bpe_path=BPE_PATH)
    tokens = [tokenizer(t).unsqueeze(0).to(device) for t in text]
    tokens = torch.cat(tokens, dim=0)
    return tokens

def load_and_transform_vision_data(cv_images, device):
    image_outputs = []

    data_transform = transforms.Compose(
        [
            transforms.Resize(
                224, interpolation=transforms.InterpolationMode.BICUBIC
            ),
            transforms.CenterCrop(224),
            transforms.ToTensor(),
            transforms.Normalize(
                mean=(0.48145466, 0.4578275, 0.40821073),
                std=(0.26862954, 0.26130258, 0.27577711),
            ),
        ]
    )

    for cv_image in cv_images:
        image = PILImage.fromarray(cv_image).convert("RGB")
        image = data_transform(image).to(device)
        image_outputs.append(image)
    return torch.stack(image_outputs, dim=0)

class ImageBindInferenceNode(object):
    def __init__(self):
        super(ImageBindInferenceNode, self).__init__()
        self.device = rospy.get_param(
            "~device", "cuda" if torch.cuda.is_available() else "cpu"
        )
        self.model = imagebind_model.imagebind_huge(pretrained=True).to(self.device)
        self.model.eval()
        self.reconfigure_server = Server(ServerConfig, self.config_cb)
        self.bridge = CvBridge()

        self.sub_image = rospy.Subscriber(
            "~input_image",
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24,
        )

        self.pub_debug_image = rospy.Publisher("~debug_image", Image, queue_size=1)
        self.pub_clip_result = rospy.Publisher(
            "~result", ClipResultStamped, queue_size=1
        )

    def config_cb(self, config, level):
        self.desc = [
            _desc.strip() for _desc in config.description.split(";") if _desc.strip()
        ]
        rospy.loginfo(f"Descriptions: {self.desc}")
        self.text_input = load_and_transform_text(self.desc, self.device)
        return config

    @torch.no_grad()
    def image_callback(self, image_msg):
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        image_input = load_and_transform_vision_data([cv_image] * len(self.desc), self.device)

        inputs = {
            ModalityType.TEXT: self.text_input,
            ModalityType.VISION: image_input,
        }

        embeddings = self.model(inputs)

        similarity = (embeddings[ModalityType.VISION] @ embeddings[ModalityType.TEXT].T).softmax(dim=-1).cpu().numpy()
        clip_result_msg = ClipResultStamped()
        clip_result_msg.header = image_msg.header
        clip_result_msg.clip.image = image_msg

        # draw text on the image with cv2
        for i, desc in enumerate(self.desc):
            clip_result_msg.clip.result.append(
                QueryAndProbability(query=desc, probability=similarity[0][i])
            )
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

        image_with_similarity_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        self.pub_debug_image.publish(image_with_similarity_msg)
        self.pub_clip_result.publish(clip_result_msg)


if __name__ == "__main__":
    rospy.init_node("imagebind_inference_node")
    node = ImageBindInferenceNode()
    rospy.spin()
