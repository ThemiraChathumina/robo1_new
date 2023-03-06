import openai

openai.api_key = "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"


def generate_prompt(animal):
    return "write a poem with 4 lines for my girlfriend. "


response = openai.Completion.create(
    model="text-davinci-003",
    prompt=generate_prompt("dog"),
    max_tokens=1000,
    temperature=0.6
)

print(response["choices"][0]["text"])
