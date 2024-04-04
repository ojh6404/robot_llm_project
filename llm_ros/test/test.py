#!/usr/bin/env python3
import os
import cv2
import base64
from openai import OpenAI
import numpy as np
from typing import Union

# Logging
import logging
logger = logging.getLogger(os.path.basename(__file__))
logger.setLevel(logging.INFO)
stream_handler = logging.StreamHandler()
stream_handler.setLevel(logging.INFO)
handler_format = logging.Formatter('%(asctime)s : [%(name)s - %(lineno)d] %(levelname)-8s - %(message)s')
stream_handler.setFormatter(handler_format)
logger.addHandler(stream_handler)

class RunGPT4OnImage():
  def __init__(self, api_key: Union[str, None]=None, model: str='gpt-4-vision-preview', max_tokens_per_call: int=200) -> None:
    # GPT-4 with vision
    self.OPENAI_API_KEY=api_key if not api_key is None else os.environ['OPENAI_API_KEY']
    self.client = OpenAI(api_key=self.OPENAI_API_KEY)
    self.payload = {
      'model': model,
      'messages': [], # ここに質問や画像などを並べてモデルに投げる。
      'max_tokens': max_tokens_per_call,
    }

  def encode_image_path(self, input_image_path: str) -> str:
    with open(input_image_path, 'rb') as image_file:
      return base64.b64encode(image_file.read()).decode('utf-8')

  def encode_image_array(self, input_image: np.ndarray) -> str:
    ret, buffer = cv2.imencode('.png', input_image)
    if ret:
      return base64.b64encode(buffer).decode('utf-8')

  def add_message_entry_as_specified_role(self, role: str) -> None:
    self.payload['messages'].append({'role': role, 'content': []})

  def add_text_content(self, text: str):
    self.payload['messages'][0]['content'].append(
      {'type': 'text', 'text': text}
    )

  def add_urlimage_content(self, urlimage: str, detail: str='auto') -> None:
    self.payload['messages'][0]['content'].append(
      {'type': 'image_url', 'image_url': {'url': urlimage}, 'detail': detail}
    )

  def add_b64image_content(self, b64image: str, detail: str='auto') -> None:
    self.payload['messages'][0]['content'].append(
      {'type': 'image_url', 'image_url': {'url': f'data:image/jpeg;base64,{b64image}', 'detail': detail}}
    )

  def add_content(self, contents: dict, as_type: str) -> None:
    if as_type == 'text':
      self.add_text_content(input_message_in_ja=contents['message'], glossary_name=contents['glossary'])
    if as_type == 'urlimage':
      self.add_urlimage_content(url=contents['url'], detail=contents['details'])
    if as_type == 'b64image':
      self.add_b64image_content(b64image=contents['b64image'], detail=contents['details'])

  def delete_messages(self) -> None:
    self.payload['messages'] = []

  def delete_content(self, index: int=-1) -> None:
    del self.payload['messages'][0]['content'][index]

  def print_payload(self) -> dict:
    logger.info(self.payload)
    return self.payload

  def execute(self) -> str:
    result = self.client.chat.completions.create(**self.payload)

    logger.info(f'Finish reason: {result.choices[0].finish_reason}')
    logger.info(f'Created: {result.created}')
    logger.info(f'ID: {result.id}')
    logger.info(f'Usage')
    logger.info(f'  Completion tokens: {result.usage.completion_tokens}')
    logger.info(f'  Prompt tokens: {result.usage.prompt_tokens}')
    logger.info(f'  Total tokens: {result.usage.total_tokens}')
    logger.info(f'Model output: {result.choices[0].message.content}')

    return result.choices[0].message.content

if __name__ == '__main__':
  input_image_path = './desk.jpg'

  gpt4v = RunGPT4OnImage()
  gpt4v.add_message_entry_as_specified_role(role='user')
  gpt4v.add_text_content(text='What do you see in this image?')
  b64image = gpt4v.encode_image_path(input_image_path=input_image_path)
  gpt4v.add_b64image_content(b64image=b64image)
  _ = gpt4v.execute()
