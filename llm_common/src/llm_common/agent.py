import os
import queue
from typing import List, Union

from openai import OpenAI
import tiktoken

from prompt.prompts import SYSTEM_PROMPT, PRIMITIVES

enc = tiktoken.get_encoding("cl100k_base")


class GPTAgent(object):
    def __init__(self, model: str = "gpt-4-vision-preview") -> None:
        self.model = model
        self.client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
        self.stop = None
        self.seed = 123
        self.logprobs = True
        self.top_logprobs = 2

        self.messages = []
        self.messages.append({"role": "system", "content": SYSTEM_PROMPT + PRIMITIVES})
        self.max_token_length = 8000
        self.max_completion_length = 2000
        self.last_response = None

    def query(
        self,
        prompt: str,
        temperature: float = 0.0,
    ) -> Union[str, None]:
        # if len(enc.encode(prompt_content)) > self.max_token_length - \
        #         self.max_completion_length:
        print(self.messages)
        try:
            messages = self.messages + [{"role": "user", "content": prompt}]
            response = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                max_tokens=self.max_completion_length,
                temperature=temperature,
                top_p=0.5,
                frequency_penalty=0.0,
                presence_penalty=0.0,
                stop=self.stop,
                seed=self.seed,
                logprobs=self.logprobs,
                top_logprobs=self.top_logprobs,
            )
            answer = response.choices[0].message.content
            return answer
        except Exception as e:
            print(f"An error occurred: {e}")
            return None

    def reset_messages(self) -> None:
        self.messages = []

    def create_prompt(self) -> str:
        pass

    def task_parser(self, task: str) -> queue.Queue:
        """
        task is like ["DETECT(object)", REACH(object), GRASP()]
        """
        print("Parsing task...")


if __name__ == "__main__":
    task = "pick up the dish from the table and put it in the dishwasher."
    agent = GPTAgent(model="gpt-4")
    response_text = agent.query(task)
    print(response_text)
