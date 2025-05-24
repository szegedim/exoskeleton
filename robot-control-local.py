import os
import re
import math
import random
import time
import numpy as np

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

# Function to load dataset from robot-control.txt
def load_dataset():
    try:
        with open("robot-control.txt", "r", encoding="utf-8") as file:
            lines = file.readlines()
        
        # Skip the header line
        data_lines = lines[1:] if len(lines) > 1 else []
        
        # Parse each line into a data entry
        dataset = []
        for line in data_lines:
            values = line.strip().split('\t')
            if len(values) >= 8:  # Ensure we have all columns
                entry = {
                    'prev_theta1': float(values[0]),
                    'prev_theta2': float(values[1]),
                    'start_theta1': float(values[2]),
                    'start_theta2': float(values[3]),
                    'end_theta1': float(values[4]),
                    'end_theta2': float(values[5]),
                    'tau1': float(values[6]),
                    'tau2': float(values[7])
                }
                dataset.append(entry)
        
        return dataset
    
    except Exception as e:
        print(f"Error loading dataset: {str(e)}")
        return []

# Function to find the best matching thetas in the dataset
def find_best_match(dataset, prev_theta1, prev_theta2, start_theta1, start_theta2, end_theta1, end_theta2):
    if not dataset:
        return None
    
    # Create array of query thetas
    query = np.array([prev_theta1, prev_theta2, start_theta1, start_theta2, end_theta1, end_theta2])
    
    best_match = None
    min_distance = float('inf')
    
    # Find the closest match using Euclidean distance
    for entry in dataset:
        # Create array of entry thetas
        entry_thetas = np.array([
            entry['prev_theta1'], entry['prev_theta2'],
            entry['start_theta1'], entry['start_theta2'],
            entry['end_theta1'], entry['end_theta2']
        ])
        
        # Calculate Euclidean distance
        distance = np.sqrt(np.sum((query - entry_thetas)**2))
        
        # Update best match if this is closer
        if distance < min_distance:
            min_distance = distance
            best_match = entry
    
    return best_match

# Main simulation function
def simulate_arm(max_steps=1000):
    global theta1, theta2, omega1, omega2
    
    # Load the dataset
    dataset = load_dataset()
    if not dataset:
        print("Error: Could not load dataset. Using gravitational torques only.")
    
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
        
        # Use local search to find best matching torques
        if dataset:
            best_match = find_best_match(
                dataset, 
                prev_theta1, prev_theta2, 
                start_theta1, start_theta2, 
                theta1, theta2
            )
            
            if best_match:
                tau1 = best_match['tau1']
                tau2 = best_match['tau2']
            else:
                # Fallback to gravitational torques
                tau1, tau2 = compute_gravitational_torques(theta1, theta2)
        else:
            # If dataset couldn't be loaded, use gravitational torques
            tau1, tau2 = compute_gravitational_torques(theta1, theta2)
        
        # Update state
        theta1, omega1, theta2, omega2 = simulate_step(theta1, omega1, theta2, omega2, tau1, tau2)
        
        # Print state
        print(f"{prev_theta1:.6f}\t{prev_theta2:.6f}\t{start_theta1:.6f}\t{start_theta2:.6f}\t{theta1:.6f}\t{theta2:.6f}\t{tau1:.2f}\t{tau2:.2f}")
        
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

# Main function
if __name__ == "__main__":
    # Initialize random seed
    random.seed(int(time.time()))
    
    # Set initial conditions - 30° for both rods (π/6 radians)
    theta1 = math.pi / 6
    theta2 = math.pi / 6
    
    # Run simulation
    simulate_arm()