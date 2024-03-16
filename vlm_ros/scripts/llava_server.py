#!/usr/bin/env python
# -*- coding: utf-8 -*-

import torch
from PIL import Image
import re
import base64
import numpy as np
import cv2
from flask import Flask, request, jsonify
from argparse import ArgumentParser

from llava.constants import (
    IMAGE_TOKEN_INDEX,
    DEFAULT_IMAGE_TOKEN,
    DEFAULT_IM_START_TOKEN,
    DEFAULT_IM_END_TOKEN,
    IMAGE_PLACEHOLDER,
)
from llava.conversation import conv_templates, SeparatorStyle
from llava.model.builder import load_pretrained_model
from llava.utils import disable_torch_init
from llava.mm_utils import (
    process_images,
    tokenizer_image_token,
    get_model_name_from_path,
    KeywordsStoppingCriteria,
)

def parse_args():
    args = ArgumentParser()
    args.add_argument("--model_name", type=str, default="7B", help="model name to load", choices=["7B", "13B"])
    args.add_argument("--load_in_8bit", action="store_true", help="load in 8bit")
    args.add_argument("--load_in_4bit", action="store_true", help="load in 4bit")
    args.add_argument("--device", type=str, default="cuda", help="device to use for inference")
    return args.parse_args()

args = parse_args()
model_path = "liuhaotian/llava-v1.5-7b" if args.model_name == "7B" else "liuhaotian/llava-v1.5-13b"
model_name = get_model_name_from_path(model_path)
if args.load_in_4bit:
    args.load_in_8bit = False
print(f"Model loaded from {model_path}")

disable_torch_init() # faster loading
tokenizer, model, image_processor, context_len = load_pretrained_model(
    model_path=model_path,
    model_base=None,
    model_name=model_name,
    load_8bit=args.load_in_8bit,
    load_4bit=args.load_in_4bit,
    device_map="auto",
    device=args.device,
)


def infer(query, cvimg, **kwargs):
    qs = query
    conv_mode = kwargs.get("conv_mode", None)
    image_token_se = DEFAULT_IM_START_TOKEN + DEFAULT_IMAGE_TOKEN + DEFAULT_IM_END_TOKEN
    if IMAGE_PLACEHOLDER in qs:
        if model.config.mm_use_im_start_end:
            qs = re.sub(IMAGE_PLACEHOLDER, image_token_se, qs)
        else:
            qs = re.sub(IMAGE_PLACEHOLDER, DEFAULT_IMAGE_TOKEN, qs)
    else:
        if model.config.mm_use_im_start_end:
            qs = image_token_se + "\n" + qs
        else:
            qs = DEFAULT_IMAGE_TOKEN + "\n" + qs

    if "llama-2" in model_name.lower():
        conv_mode = "llava_llama_2"
    elif "v1" in model_name.lower():
        conv_mode = "llava_v1"
    elif "mpt" in model_name.lower():
        conv_mode = "mpt"
    else:
        conv_mode = "llava_v0"

    if conv_mode is not None and conv_mode != conv_mode:
        print(
            "[WARNING] the auto inferred conversation mode is {}, while `--conv-mode` is {}, using {}".format(
                conv_mode, conv_mode, conv_mode
            )
        )
    else:
        conv_mode = conv_mode

    conv = conv_templates[conv_mode].copy()
    conv.append_message(conv.roles[0], qs)
    conv.append_message(conv.roles[1], None)
    prompt = conv.get_prompt()

    images = [Image.fromarray(cv2.cvtColor(cvimg, cv2.COLOR_BGR2RGB))]  # TODO : batch
    images_tensor = process_images(images, image_processor, model.config).to(model.device, dtype=torch.float16)
    input_ids = tokenizer_image_token(prompt, tokenizer, IMAGE_TOKEN_INDEX, return_tensors="pt").unsqueeze(0).cuda()

    stop_str = conv.sep if conv.sep_style != SeparatorStyle.TWO else conv.sep2
    keywords = [stop_str]
    stopping_criteria = KeywordsStoppingCriteria(keywords, tokenizer, input_ids)

    with torch.inference_mode():
        output_ids = model.generate(
            input_ids,
            images=images_tensor,
            do_sample=kwargs["do_sample"],
            temperature=kwargs["temperature"],
            top_p=kwargs["top_p"],
            num_beams=kwargs["num_beams"],
            max_new_tokens=kwargs["max_new_tokens"],
            use_cache=True,
            stopping_criteria=[stopping_criteria],
        )

    input_token_len = input_ids.shape[1]
    n_diff_input_output = (input_ids != output_ids[:, :input_token_len]).sum().item()
    if n_diff_input_output > 0:
        print(f"[Warning] {n_diff_input_output} output_ids are not the same as the input_ids")
    outputs = tokenizer.batch_decode(output_ids[:, input_token_len:], skip_special_tokens=True)[0]
    outputs = outputs.strip()
    if outputs.endswith(stop_str):
        outputs = outputs[: -len(stop_str)]
    outputs = outputs.strip()
    return outputs


def decode_image(img):
    img = base64.b64decode(img)
    npimg = np.frombuffer(img, dtype=np.uint8)
    cvimg = cv2.imdecode(npimg, cv2.IMREAD_COLOR)
    return cvimg


def process_config(config):
    config["top_p"] = config.get("top_k", 0)
    if "top_k" in config:
        del config["top_k"]
    config["max_new_tokens"] = config.get("max_length", 512)
    if "max_length" in config:
        del config["max_length"]
    config["temperature"] = config.get("temperature", 0)
    config["do_sample"] = True if config["temperature"] > 0 else False
    config["num_beams"] = config.get("num_beams", 1)
    config["conv_mode"] = config.get("conv_mode", None)
    return config


if __name__ == "__main__":
    app = Flask(__name__)
    try:

        @app.route("/text_gen", methods=["POST"])
        def text_gen_request():
            data = request.get_json()
            gen_config = data["gen_config"]
            gen_config = process_config(gen_config)
            img = data["image"].encode("utf-8")
            cvimg = decode_image(img)
            queries = data["queries"]  # TODO : batch
            query = queries[0]
            sentences = infer(query, cvimg, **gen_config)
            sentences = [sentences]
            response = {"queries": queries, "answeres": sentences}
            return jsonify(response)

    except NameError:
        print("Skipping create text_gen app")

    app.run("0.0.0.0", 8080, threaded=True)
