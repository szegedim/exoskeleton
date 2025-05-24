import os
import re
import math
import random
import time
from cerebras.cloud.sdk import Cerebras

# This document is Licensed under Creative Commons CC0.
# To the extent possible under law, the author(s) have dedicated all copyright and related and neighboring rights
# to this document to the public domain worldwide.
# This document is distributed without any warranty.
# You should have received a copy of the CC0 Public Domain Dedication along with this document.
# If not, see https://creativecommons.org/publicdomain/zero/1.0/legalcode.

# Constants (same as in C code)
G = 9.81       # Gravity (m/s²)
L1 = 1.0       # Length of first rod (m)
L2 = 1.5       # Length of second rod (m)
M1 = 1.0       # Mass of first rod (kg)
M2 = 1.5       # Mass of second rod (kg)
DT = 0.01      # Time step (s)
KP1 = 50.0     # Proportional gain for joint 1
KD1 = 20.0     # Derivative gain for joint 1
KP2 = 50.0     # Proportional gain for joint 2
KD2 = 20.0     # Derivative gain for joint 2

# State variables
theta1 = 0.0   # Angle of first rod (radians)
theta2 = 0.0   # Angle of second rod (radians)
omega1 = 0.0   # Angular velocity of first rod (rad/s)
omega2 = 0.0   # Angular velocity of second rod (rad/s)

# Function to compute gravitational torque for each joint
def compute_gravitational_torques(theta1, theta2):
    # Gravitational torque on first rod (negative when rod is at positive angle)
    tau1 = -M1 * G * L1 * math.sin(theta1) - M2 * G * L1 * math.sin(theta1)
    # Gravitational torque on second rod (negative when rod is at positive angle)
    tau2 = -M2 * G * L2 * math.sin(theta2)
    return tau1, tau2

# Function to simulate one time step
def simulate_step(theta1, omega1, theta2, omega2, tau1, tau2):
    # Angular accelerations (τ = Iα, I = mL² for each rod)
    alpha1 = tau1 / (M1 * L1 * L1)  # First joint
    alpha2 = tau2 / (M2 * L2 * L2)  # Second joint

    # Update angular velocities and angles
    omega1 += alpha1 * DT
    omega2 += alpha2 * DT
    theta1 += omega1 * DT
    theta2 += omega2 * DT
    
    return theta1, omega1, theta2, omega2

# Improved parsing for the torque values
def parse_torques(response_text):
    # First try the standard regex pattern for numbers
    numbers = re.findall(r'-?\d+\.?\d*', response_text)
    
    if len(numbers) >= 2:
        return float(numbers[0]), float(numbers[1])
    
    # If standard pattern fails, try other formats
    # Try comma-separated pattern (e.g. "-10.5, 15.3")
    comma_pattern = re.search(r'(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)', response_text)
    if comma_pattern:
        return float(comma_pattern.group(1)), float(comma_pattern.group(2))
    
    # Try newline-separated pattern (e.g. "-10.5\n15.3")
    newline_pattern = re.search(r'(-?\d+\.?\d*)\s*[\n\r]+\s*(-?\d+\.?\d*)', response_text)
    if newline_pattern:
        return float(newline_pattern.group(1)), float(newline_pattern.group(2))
    
    # Try tab-separated pattern (e.g. "-10.5\t15.3")
    tab_pattern = re.search(r'(-?\d+\.?\d*)\s*\t\s*(-?\d+\.?\d*)', response_text)
    if tab_pattern:
        return float(tab_pattern.group(1)), float(tab_pattern.group(2))
    
    # If all patterns fail, return None
    return None

# Main simulation function
def simulate_arm(api_client, max_steps=1000):
    global theta1, theta2, omega1, omega2
    
    # Load the dataset from robot-control.txt
    try:
        with open("robot-control.txt", "r", encoding="utf-8") as file:
            dataset = file.read().strip()
    except FileNotFoundError:
        print("Warning: robot-control.txt not found. Using empty dataset.")
        dataset = ""
    
    target_theta1 = 0.0  # Target angle for first rod (0°)
    target_theta2 = 0.0  # Target angle for second rod (0°)
    
    # Track previous theta values
    prev_theta1 = theta1
    prev_theta2 = theta2
    
    # Print header
    print("Prev_Theta1\tPrev_Theta2\tStart_Theta1\tStart_Theta2\tEnd_Theta1\tEnd_Theta2\tTorque1\tTorque2")
    
    simulation_data = []
    
    for i in range(max_steps):
        # Store initial state for this step
        start_theta1 = theta1
        start_theta2 = theta2
        start_omega1 = omega1
        start_omega2 = omega2
        
        # Create the question with the theta values
        question_base = (f"I give a reference line and a data table. Find the closest line of theta numbers as key in the table for the reference number. "
                        f"Give me the torques. Give me just the two numbers, nothing else.\n"
                        f"Prev_Theta1\tPrev_Theta2\tStart_Theta1\tStart_Theta2\tEnd_Theta1\tEnd_Theta2\n"
                        f"{prev_theta1:.6f}\t{prev_theta2:.6f}\t{start_theta1:.6f}\t{start_theta2:.6f}\t{theta1:.6f}\t{theta2:.6f}")
        # print(f"Question: {question_base}")
        # Append the dataset to the question
        full_question = question_base + "\n\n" + dataset
        
        # Make API call to get torques using streaming
        response = api_client.chat.completions.create(
            messages=[
                {
                    "role": "system",
                    "content": "Give me just the two torques, nothing else. Do not include thinking between tags in the answer."
                },
                {
                    "role": "user",
                    "content": full_question
                }
            ],
            model="llama-4-scout-17b-16e-instruct",
            stream=True,  # Enable streaming
            max_completion_tokens=8192,
            temperature=0,
            top_p=1
        )

        # Handle streaming response
        response_text = ""
        # print("Streaming response: ", end="", flush=True)
        for chunk in response:
            if hasattr(chunk.choices[0].delta, 'content') and chunk.choices[0].delta.content:
                content_chunk = chunk.choices[0].delta.content
                response_text += content_chunk
                print(content_chunk, end="", flush=True)

        print()  # Add a newline after the response finishes

        # Use the enhanced parser
        torques = parse_torques(response_text)
        if torques:
            tau1, tau2 = torques
        else:
            # Fallback to gravitational torques if parsing fails
            tau1, tau2 = compute_gravitational_torques(theta1, theta2)
            print(f"Warning: Could not parse torques from response: {response_text}")
        
        # Update state
        theta1, omega1, theta2, omega2 = simulate_step(theta1, omega1, theta2, omega2, tau1, tau2)
        
        # Print state
        # print(f"{prev_theta1:.6f}\t{prev_theta2:.6f}\t{start_theta1:.6f}\t{start_theta2:.6f}\t{theta1:.6f}\t{theta2:.6f}\t{tau1:.2f}\t{tau2:.2f}")
        
        # Save the data for potential future use
        simulation_data.append({
            'prev_theta1': prev_theta1, 'prev_theta2': prev_theta2,
            'start_theta1': start_theta1, 'start_theta2': start_theta2,
            'end_theta1': theta1, 'end_theta2': theta2,
            'tau1': tau1, 'tau2': tau2
        })
        
        # Update previous thetas for next iteration
        prev_theta1 = start_theta1
        prev_theta2 = start_theta2
        
        # Stop if close to target
        if (abs(theta1-target_theta1) < 0.01 and abs(omega1) < 0.01 and 
            abs(theta2-target_theta2) < 0.01 and abs(omega2) < 0.01):
            break
    
    return simulation_data

# Function to get API key from file
def get_api_key():
    try:
        # Expand the ~ to the user's home directory
        key_path = os.path.expanduser("~/.ssh/cerebras")
        with open(key_path, "r") as key_file:
            return key_file.read().strip()
    except FileNotFoundError:
        print(f"Error: API key file not found at ~/.ssh/cerebras")
        print(f"Please create this file with your Cerebras API key")
        exit(1)
    except Exception as e:
        print(f"Error reading API key: {e}")
        exit(1)

# Main function
if __name__ == "__main__":
    # Initialize random seed
    random.seed(int(time.time()))
    
    # Get API key from file
    cerebras_api_key = get_api_key()
    
    # Initialize Cerebras client
    client = Cerebras(
        api_key=cerebras_api_key
    )
    
    # Set initial conditions - 30° for both rods (π/6 radians)
    theta1 = math.pi / 6
    theta2 = math.pi / 6
    
    # Run simulation
    simulate_arm(client)
