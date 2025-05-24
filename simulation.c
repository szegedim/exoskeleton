#include <stdio.h>
#include <math.h>
#include <stdlib.h> // For rand() and srand()
#include <time.h>   // For time()

// This document is Licensed under Creative Commons CC0.
// To the extent possible under law, the author(s) have dedicated all copyright and related and neighboring rights
// to this document to the public domain worldwide.
// This document is distributed without any warranty.
// You should have received a copy of the CC0 Public Domain Dedication along with this document.
// If not, see https://creativecommons.org/publicdomain/zero/1.0/legalcode.

/*
gcc simulation.c -o simulation -lSDL2 -lm $(sdl2-config --cflags --libs); ./simulation
*/

// Constants
#define G 9.81       // Gravity (m/s²)
#define L1 1.0       // Length of first rod (m)
#define L2 1.5       // Length of second rod (m)
#define M1 1.0       // Mass of first rod (kg)
#define M2 1.5       // Mass of second rod (kg)
#define DT 0.01      // Time step (s)
#define KP1 50.0     // Proportional gain for joint 1
#define KD1 20.0     // Derivative gain for joint 1
#define KP2 50.0     // Proportional gain for joint 2
#define KD2 20.0     // Derivative gain for joint 2

// State variables
double theta1 = 0.0;  // Angle of first rod (radians)
double omega1 = 0.0;  // Angular velocity of first rod (rad/s)
double theta2 = 0.0;  // Angle of second rod (radians)
double omega2 = 0.0;  // Angular velocity of second rod (rad/s)

// Function to compute gravitational torque for each joint
void compute_gravitational_torques(double theta1, double theta2, double *tau1, double *tau2) {
    // Gravitational torque on first rod (negative when rod is at positive angle)
    *tau1 = -M1 * G * L1 * sin(theta1) - M2 * G * L1 * sin(theta1); // Second rod's mass affects first joint
    // Gravitational torque on second rod (negative when rod is at positive angle)
    *tau2 = -M2 * G * L2 * sin(theta2);
}

// Function to compute control torque using PD controller
void compute_control_torques(double theta1, double omega1, double theta2, double omega2, double *tau1, double *tau2) {
    // Error-correcting torques (negative feedback for stability)
    double control_tau1 = -KP1 * theta1 - KD1 * omega1; 
    double control_tau2 = -KP2 * theta2 - KD2 * omega2;
    
    // Add random noise of ±10% to simulate real-world conditions
    double noise_factor1 = 1.0 + (rand() % 201 - 100) / 1000.0; // Range: 0.9 to 1.1
    double noise_factor2 = 1.0 + (rand() % 201 - 100) / 1000.0; // Range: 0.9 to 1.1
    
    control_tau1 *= noise_factor1;
    control_tau2 *= noise_factor2;
    
    // Add control torque to gravitational torque
    *tau1 += control_tau1;
    *tau2 += control_tau2;
}

// Function to simulate one time step
void simulate_step(double *theta1, double *omega1, double *theta2, double *omega2, double tau1, double tau2) {
    // Angular accelerations (τ = Iα, I = mL² for each rod)
    double alpha1 = (tau1) / (M1 * L1 * L1); // First joint
    double alpha2 = (tau2) / (M2 * L2 * L2); // Second joint

    // Update angular velocities and angles
    *omega1 += alpha1 * DT;
    *omega2 += alpha2 * DT;
    *theta1 += *omega1 * DT;
    *theta2 += *omega2 * DT;
}

// Main simulation loop
void simulate_arm() {
    double target_theta1 = 0.0; // Target angle for first rod (0°)
    double target_theta2 = 0.0; // Target angle for second rod (0°)
    double max_steps = 1000;    // Simulate for 10 seconds (1000 steps at 0.01s/step)

    // Track previous theta values
    double prev_theta1 = theta1;
    double prev_theta2 = theta2;

    // Updated header without time
    printf("Prev_Theta1\tPrev_Theta2\tStart_Theta1\tStart_Theta2\tEnd_Theta1\tEnd_Theta2\tTorque1\tTorque2\n");
    
    for (int i = 0; i < max_steps; i++) {
        // Store initial state for this step
        double start_theta1 = theta1;
        double start_theta2 = theta2;
        double start_omega1 = omega1;
        double start_omega2 = omega2;

        // Calculate torques
        double tau1 = 0.0, tau2 = 0.0;
        compute_gravitational_torques(theta1, theta2, &tau1, &tau2);
        compute_control_torques(theta1, omega1, theta2, omega2, &tau1, &tau2);

        // Update state
        simulate_step(&theta1, &omega1, &theta2, &omega2, tau1, tau2);

        // Print only theta values and torques (no time, no omegas)
        printf("%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.2f\t%.2f\n", 
               prev_theta1, prev_theta2,
               start_theta1, start_theta2,
               theta1, theta2,
               tau1, tau2);
        
        // Update previous thetas for next iteration
        prev_theta1 = start_theta1;
        prev_theta2 = start_theta2;

        // Stop if close to target
        if (fabs(theta1-target_theta1) < 0.01 && fabs(omega1) < 0.01 && 
            fabs(theta2-target_theta2) < 0.01 && fabs(omega2) < 0.01) break;
    }
}

int main() {
    // Initialize random seed using current time
    srand((unsigned int)time(NULL));
    
    // Example: Start at 30° for both rods (π/6 radians)
    theta1 = M_PI / 6;
    theta2 = M_PI / 6;
    simulate_arm();
    return 0;
}