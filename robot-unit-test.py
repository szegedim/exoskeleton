import requests
import json
import os
from pathlib import Path

# This document is Licensed under Creative Commons CC0.
# To the extent possible under law, the author(s) have dedicated all copyright and related and neighboring rights
# to this document to the public domain worldwide.
# This document is distributed without any warranty.
# You should have received a copy of the CC0 Public Domain Dedication along with this document.
# If not, see https://creativecommons.org/publicdomain/zero/1.0/legalcode.

# Function to get API key from file
def get_api_key():
    try:
        # Expand the ~ to the user's home directory
        key_path = os.path.expanduser("~/.ssh/openrouter")
        with open(key_path, "r") as key_file:
            return key_file.read().strip()
    except FileNotFoundError:
        print(f"Error: API key file not found at ~/.ssh/openrouter")
        print(f"Please create this file with your OpenRouter API key")
        exit(1)
    except Exception as e:
        print(f"Error reading API key: {e}")
        exit(1)

# Read the question from robot-test.txt
try:
    with open("robot-test.txt", "r", encoding="utf-8") as file:
        original_content = file.read().strip()
except FileNotFoundError:
    print("Error: robot-test.txt file not found. Creating a sample file...")
    with open("robot-test.txt", "w", encoding="utf-8") as file:
        file.write("Provide the torques for these values.")
    original_content = "Provide the torques for these values."
    print(f"Created robot-test.txt with default content")

# Prepend the command and theta values to the question
command = "Find the best matching line for these theta numbers in the dataset below. Give me the torques. Give me just the two numbers, nothing else.\n0.551393\t0.531838\t0.553141\t0.532356\t0.553141\t0.532356\n\n"
question = command + original_content

# Get API key from file
api_key = get_api_key()

url = "https://openrouter.ai/api/v1/chat/completions"
headers = {
  "Authorization": f"Bearer {api_key}",
  "Content-Type": "application/json"
}

payload = {
  "model": "anthropic/claude-3-haiku",  # Changed to Claude for better parsing of tables
  "messages": [
    {
      "role": "system", 
      "content": "Extract only the two torque values from the dataset that best match the given theta values. Respond with only the two numbers separated by a space or tab."
    },
    {"role": "user", "content": question}
  ],
  "stream": True,
  "temperature": 0.1  # Lower temperature for more precise responses
}

print(f"Searching for torques matching: 0.551393\t0.531838\t0.553141\t0.532356\t0.553141\t0.532356\n")

buffer = ""
full_response = ""  # Store the complete response
with requests.post(url, headers=headers, json=payload, stream=True) as r:
  for chunk in r.iter_content(chunk_size=1024, decode_unicode=True):
    buffer += chunk
    while True:
      try:
        # Find the next complete SSE line
        line_end = buffer.find('\n')
        if line_end == -1:
          break

        line = buffer[:line_end].strip()
        buffer = buffer[line_end + 1:]

        if line.startswith('data: '):
          data = line[6:]
          if data == '[DONE]':
            break

          try:
            data_obj = json.loads(data)
            content = data_obj["choices"][0]["delta"].get("content")
            if content:
              print(content, end="", flush=True)
              full_response += content  # Add to full response
          except json.JSONDecodeError:
            pass
      except Exception:
        break

print("\n")  # Add a line break after the response

# Check if the response matches the expected value
expected_response = "4.55 2.48"
# Clean up response by removing extra whitespace and normalizing
cleaned_response = ' '.join(full_response.strip().split())

if cleaned_response == expected_response:
    print("PASS")
else:
    print(f"FAIL: Expected '{expected_response}', got '{cleaned_response}'")

