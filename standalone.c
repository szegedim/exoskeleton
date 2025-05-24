#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <SDL2/SDL.h>

// This document is Licensed under Creative Commons CC0.
// To the extent possible under law, the author(s) have dedicated all copyright and related and neighboring rights
// to this document to the public domain worldwide.
// This document is distributed without any warranty.
// You should have received a copy of the CC0 Public Domain Dedication along with this document.
// If not, see https://creativecommons.org/publicdomain/zero/1.0/legalcode.

/*
gcc standalone.c -o standalone -lSDL2 -lm $(sdl2-config --cflags --libs); ./standalone
*/

// Physics constants (from cerebras.c)
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

// Display constants (from simulation.c)
#define SCREEN_WIDTH 800
#define SCREEN_HEIGHT 600
#define WINDOW_TITLE "Humanoid Physics Simulation"
#define ROD_LENGTH_SCALE 100.0f  // Pixels per meter

// State variables
double theta1 = 0.0;  // Angle of first rod (radians)
double omega1 = 0.0;  // Angular velocity of first rod (rad/s)
double theta2 = 0.0;  // Angle of second rod (radians)
double omega2 = 0.0;  // Angular velocity of second rod (rad/s)
double tau1 = 0.0;    // Torque on first joint
double tau2 = 0.0;    // Torque on second joint

// Previous values for display
double prev_theta1 = 0.0;
double prev_theta2 = 0.0;

// Graphics variables
SDL_Window* window = NULL;
SDL_Renderer* renderer = NULL;
SDL_Texture* data_texture = NULL;
SDL_Rect data_rect;
int font_height = 16;

// Function to compute gravitational torque for each joint
void compute_gravitational_torques(double theta1, double theta2, double *tau1, double *tau2) {
    // Gravitational torque on first rod (negative when rod is at positive angle)
    *tau1 = -M1 * G * L1 * sin(theta1) - M2 * G * L1 * sin(theta1);
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

// Initialize SDL
int initialize_graphics() {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        printf("SDL could not initialize! SDL_Error: %s\n", SDL_GetError());
        return 0;
    }

    window = SDL_CreateWindow(WINDOW_TITLE,
                              SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                              SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
    if (window == NULL) {
        printf("Window could not be created! SDL_Error: %s\n", SDL_GetError());
        SDL_Quit();
        return 0;
    }

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (renderer == NULL) {
        printf("Renderer could not be created! SDL_Error: %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 0;
    }
    
    // Create a texture for rendering text
    data_rect.x = 10;
    data_rect.y = 10;
    data_rect.w = 300;
    data_rect.h = 150;
    
    return 1;
}

// Shutdown SDL
void shutdown_graphics() {
    if (data_texture != NULL) {
        SDL_DestroyTexture(data_texture);
    }
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
}

// Render the simulation
void render_simulation() {
    // Clear screen
    SDL_SetRenderDrawColor(renderer, 30, 30, 30, 255); // Dark grey background
    SDL_RenderClear(renderer);

    // Define the origin point (fixed joint)
    int origin_x = SCREEN_WIDTH / 2;
    int origin_y = SCREEN_HEIGHT / 2;

    // Calculate positions based on angles
    int joint2_x = origin_x + (int)(ROD_LENGTH_SCALE * L1 * sin(theta1));
    int joint2_y = origin_y - (int)(ROD_LENGTH_SCALE * L1 * cos(theta1));
    
    int end_x = joint2_x + (int)(ROD_LENGTH_SCALE * L2 * sin(theta1 + theta2));
    int end_y = joint2_y - (int)(ROD_LENGTH_SCALE * L2 * cos(theta1 + theta2));

    // Draw rods
    SDL_SetRenderDrawColor(renderer, 200, 200, 50, 255); // First rod: yellowish
    SDL_RenderDrawLine(renderer, origin_x, origin_y, joint2_x, joint2_y);
    
    SDL_SetRenderDrawColor(renderer, 50, 200, 200, 255); // Second rod: bluish
    SDL_RenderDrawLine(renderer, joint2_x, joint2_y, end_x, end_y);

    // Draw joints as small red circles
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255); // Red
    for (int i = -4; i <= 4; i++) {
        for (int j = -4; j <= 4; j++) {
            if (i*i + j*j <= 16) { // Circle with radius 4
                SDL_RenderDrawPoint(renderer, origin_x + i, origin_y + j);
                SDL_RenderDrawPoint(renderer, joint2_x + i, joint2_y + j);
                SDL_RenderDrawPoint(renderer, end_x + i, end_y + j);
            }
        }
    }

    // Display torque indicators
    int bar_width = 50;
    int bar_height = 10;
    
    // Torque 1 indicator
    SDL_Rect tau1_rect = {
        origin_x - bar_width/2,
        origin_y + 15,
        (int)(bar_width * fabs(tau1) / 50.0), // Scale to reasonable size
        bar_height
    };
    
    if (tau1 >= 0) {
        SDL_SetRenderDrawColor(renderer, 50, 255, 50, 255); // Green for positive
        tau1_rect.x = origin_x;
    } else {
        SDL_SetRenderDrawColor(renderer, 255, 50, 50, 255); // Red for negative
        tau1_rect.x = origin_x - tau1_rect.w;
    }
    SDL_RenderFillRect(renderer, &tau1_rect);
    
    // Torque 2 indicator
    SDL_Rect tau2_rect = {
        joint2_x - bar_width/2,
        joint2_y + 15,
        (int)(bar_width * fabs(tau2) / 50.0), // Scale to reasonable size
        bar_height
    };
    
    if (tau2 >= 0) {
        SDL_SetRenderDrawColor(renderer, 50, 255, 50, 255); // Green for positive
        tau2_rect.x = joint2_x;
    } else {
        SDL_SetRenderDrawColor(renderer, 255, 50, 50, 255); // Red for negative
        tau2_rect.x = joint2_x - tau2_rect.w;
    }
    SDL_RenderFillRect(renderer, &tau2_rect);

    // Draw data text in top-left corner
    char data_text[256];
    sprintf(data_text, "Theta1: %.4f rad\nTheta2: %.4f rad\nOmega1: %.4f rad/s\nOmega2: %.4f rad/s\nTorque1: %.2f Nm\nTorque2: %.2f Nm",
            theta1, theta2, omega1, omega2, tau1, tau2);
    
    // Render the data as a series of text lines
    int y_offset = 10;
    char *line = strtok(data_text, "\n");
    
    while (line != NULL) {
        // Create a simple text rendering
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        int x_offset = 10;
        for (int i = 0; line[i] != '\0'; i++) {
            // Very simple character rendering (just for visualization)
            for (int j = 0; j < 8; j++) {
                SDL_RenderDrawPoint(renderer, x_offset + i*8 + j, y_offset);
                SDL_RenderDrawPoint(renderer, x_offset + i*8 + j, y_offset + font_height - 1);
            }
            SDL_RenderDrawPoint(renderer, x_offset + i*8, y_offset + font_height/2);
            SDL_RenderDrawPoint(renderer, x_offset + i*8 + 7, y_offset + font_height/2);
        }
        y_offset += font_height + 2;
        line = strtok(NULL, "\n");
    }

    // Present the rendered frame
    SDL_RenderPresent(renderer);
}

// Main simulation loop
void simulate_arm_with_display() {
    double target_theta1 = 0.0; // Target angle for first rod
    double target_theta2 = 0.0; // Target angle for second rod
    int max_steps = 10000;      // Simulate for 100 seconds max
    int running = 1;
    int step = 0;
    
    // Event handling
    SDL_Event event;
    
    // Track previous theta values
    prev_theta1 = theta1;
    prev_theta2 = theta2;
    
    // Main loop
    while (running && step < max_steps) {
        // Handle events
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = 0;
            } else if (event.type == SDL_KEYDOWN) {
                if (event.key.keysym.sym == SDLK_ESCAPE || 
                    event.key.keysym.sym == SDLK_q) {
                    running = 0;
                }
            }
        }
        
        // Store initial state for this step
        double start_theta1 = theta1;
        double start_theta2 = theta2;
        double start_omega1 = omega1;
        double start_omega2 = omega2;

        // Calculate torques
        tau1 = 0.0;
        tau2 = 0.0;
        compute_gravitational_torques(theta1, theta2, &tau1, &tau2);
        compute_control_torques(theta1, omega1, theta2, omega2, &tau1, &tau2);

        // Update state
        simulate_step(&theta1, &omega1, &theta2, &omega2, tau1, tau2);

        // Render current state
        render_simulation();
        
        // Print state data to console for logging
        printf("%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.2f\t%.2f\n", 
               prev_theta1, prev_theta2,
               start_theta1, start_theta2,
               theta1, theta2,
               tau1, tau2);
        
        // Update previous thetas for next iteration
        prev_theta1 = start_theta1;
        prev_theta2 = start_theta2;
        
        // Increment step counter
        step++;
        
        // Add delay to match simulation rate with DT
        SDL_Delay((Uint32)(DT * 1000));
        
        // Stop if close to target and stable
        if (fabs(theta1-target_theta1) < 0.01 && fabs(omega1) < 0.01 && 
            fabs(theta2-target_theta2) < 0.01 && fabs(omega2) < 0.01) {
            // Wait a bit to show the final state
            SDL_Delay(2000);
            break;
        }
    }
}

int main() {
    // Initialize random seed
    srand((unsigned int)time(NULL));
    
    // Initialize graphics
    if (!initialize_graphics()) {
        return 1;
    }
    
    // Set initial conditions - 30° for both rods (π/6 radians)
    theta1 = M_PI / 6;
    theta2 = M_PI / 6;
    
    // Run simulation with display
    simulate_arm_with_display();
    
    // Cleanup
    shutdown_graphics();
    
    return 0;
}