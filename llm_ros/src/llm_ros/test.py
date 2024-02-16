import os
import openai


class QueryBase(object):
    def __init__(self, prompt: str) -> None:
        self.api_key = os.getenv("OPENAI_API_KEY")
        openai.api_key = self.api_key
        self.prompt = prompt

    def query(self) -> str:
        try:
            response = openai.Completion.create(
                engine="text-davinci-003",
                prompt=self.prompt,
                temperature=0.7,
                max_tokens=150,
                top_p=1.0,
                frequency_penalty=0.0,
                presence_penalty=0.0
            )
            return response.choices[0].text.strip()
        except Exception as e:
            print(f"An error occurred: {e}")
            return None

if __name__ == "__main__":
    prompt = "Explain the significance of the Turing Test in AI."
    query = QueryBase(prompt)
    response_text = query.query()
    print(response_text)
