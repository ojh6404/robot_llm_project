import numpy as np
import base64
import cv2

def encode_image(image: np.ndarray) -> str:
    """
    encode cv2 image to base64
    """
    base64_image = base64.b64encode(cv2.imencode(".jpg", image)[1]).decode("utf-8")
    return base64_image
