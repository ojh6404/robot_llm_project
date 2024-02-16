import os
import queue
from typing import List, Union
from openai import OpenAI

class Perception(object):
    def __init__(self) -> None:
        pass

    def get_action(self, prompt: str) -> List:
        pass

class GPTAgent(object):
    def __init__(self, model: str = "gpt-4", temperature: int = 0) -> None:
        self.model = model
        self.temperature = 0
        self.client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

    def query(self,  new_prompt: str, messages: List, role: str = "user") -> Union[List[dict], None]:
        messages.append({"role":role, "content":new_prompt})
        try:
            response = self.client.chat.completions.create(
                model=self.model,
                temperature=self.temperature,
                messages=messages,
                stream=True
            )
            new_output = ""

            for chunk in response:
                chunk_content = chunk.choices[0].delta.content
                finish_reason = chunk.choices[0].finish_reason
                if chunk_content is not None:
                    print(chunk_content, end="")
                    new_output += chunk_content
                else:
                    print("finish_reason:", finish_reason)
            messages.append({"role":"assistant", "content":new_output})
            return messages
        except Exception as e:
            print(f"An error occurred: {e}")
            return None

    def task_parser(self, task: str) -> queue.Queue:
        print("Parsing task...")


if __name__ == "__main__":
    prompt = "Explain the significance of the Turing Test in AI."
    query = GPTAgent()
    response_text = query.query(prompt, [], "user")
    print(response_text)
