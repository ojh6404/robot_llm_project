#!/usr/bin/env python
# -*- coding: utf-8 -*-

import base64
import numpy as np
import torch
import cv2
from PIL import Image
from flask import Flask, request, jsonify
import subprocess
from pathlib import Path
import requests
from tqdm import tqdm
from argparse import ArgumentParser

from pipeline.interface import get_model

MODEL_NAME = ["7B-C-Abs-M144", "7B-D-Abs-M144", "7B-C-Abs-M256", "13B-C-Abs-M256", "13B-D-Abs-M256", "13B-C-Abs-M576"]

def parse_args():
    args = ArgumentParser()
    args.add_argument("--model_name", type=str, default="7B-C-Abs-M256", help="model name to load", choices=MODEL_NAME+["7B", "13B"])
    args.add_argument("--use_bf16", action="store_true", help="use bf16")
    args.add_argument("--load_in_8bit", action="store_true", help="load in 8bit")
    args.add_argument("--device", type=str, default="cuda", help="device to use for inference")
    return args.parse_args()

def get_checkpoint_path(model_name):
    # convert model name to the correct format when given abbreviated name
    if model_name == "7B":
        model_name = "7B-C-Abs-M256"
    elif model_name == "13B":
        model_name = "13B-C-Abs-M576"
    assert model_name in MODEL_NAME, f"model_name should be one of {MODEL_NAME}"
    # download model and return path
    hf_cache = Path.home() / ".cache" / "huggingface"
    cache_root = hf_cache / "honeybee" / "checkpoints"
    cache_root.mkdir(parents=True, exist_ok=True)
    # download model if not exists
    model_path = cache_root / model_name
    if not model_path.exists():
        print(f"Downloading model {model_name} to {model_path}")
        download_url = f"https://twg.kakaocdn.net/brainrepo/models/honeybee/{model_name}.tar.gz"
        response = requests.get(download_url, stream=True)
        total_size_in_bytes = int(response.headers.get("content-length", 0))
        chunk_size = 1024  # 1 KB
        progress_bar = tqdm(total=total_size_in_bytes, unit="iB", unit_scale=True)
        with open(f"{model_path}.tar.gz", "wb") as file:
            for data in response.iter_content(chunk_size):
                progress_bar.update(len(data))
                file.write(data)
        progress_bar.close()
        subprocess.run(["tar", "-xvf", f"{model_path}.tar.gz", "-C", f"{model_path.parent}"])
        subprocess.run(["rm", f"{model_path}.tar.gz"])
    return model_path / "last"


def construct_input_prompt(user_prompt):
    # TODO : make it configurable
    SYSTEM_MESSAGE = "The following is a conversation between a curious human and AI assistant. The assistant gives helpful, detailed, and polite answers to the user's questions.\n"
    IMAGE_TOKEN = "Human: <image>\n"  # <image> denotes an image placehold.
    USER_PROMPT = f"Human: {user_prompt}\n"
    return SYSTEM_MESSAGE + IMAGE_TOKEN + USER_PROMPT + "AI: "


def infer(query, cvimg, kwargs):
    # images to PIL images
    kwargs["do_sample"] = True if kwargs.get("temperature", 0) > 0 else False
    prompts = [query]
    pilimgs = [Image.fromarray(cv2.cvtColor(cvimg, cv2.COLOR_BGR2RGB))]  # TODO : batch
    inputs = processor(text=prompts, images=pilimgs)
    inputs = {k: v.bfloat16() if v.dtype == torch.float else v for k, v in inputs.items()}
    inputs = {k: v.to(model.device) for k, v in inputs.items()}

    with torch.no_grad():
        res = model.generate(**inputs, **kwargs)
    sentences = tokenizer.batch_decode(res, skip_special_tokens=True)
    return sentences


def decode_image(img):
    img = base64.b64decode(img)
    npimg = np.frombuffer(img, dtype=np.uint8)
    cvimg = cv2.imdecode(npimg, cv2.IMREAD_COLOR)
    return cvimg

# Load trained model
args = parse_args()
model_path = get_checkpoint_path(args.model_name)
model, tokenizer, processor = get_model(model_path, use_bf16=args.use_bf16, load_in_8bit=args.load_in_8bit)
model.to(args.device)
print(f"Model loaded from {model_path}")

# run
if __name__ == "__main__":
    app = Flask(__name__)
    try:
        @app.route("/text_gen", methods=["POST"])
        def text_gen_request():
            data = request.get_json()
            gen_config = data["gen_config"]
            img = data["image"].encode("utf-8")
            cvimg = decode_image(img)
            queries = data["queries"]
            query = construct_input_prompt(queries[0])  # TODO : batch
            sentences = infer(query, cvimg, gen_config)
            response = {"queries": queries, "answeres": sentences}
            return jsonify(response)

    except NameError:
        print("Skipping create text_gen app")

    app.run("0.0.0.0", 8080, threaded=True)
