#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <SDL2/SDL.h>

// This document is Licensed under Creative Commons CC0.
// To the extent possible under law, the author(s) have dedicated all copyright and related and neighboring rights
// to this document to the public domain worldwide.
// This document is distributed without any warranty.
// You should have received a copy of the CC0 Public Domain Dedication along with this document.
// If not, see https://creativecommons.org/publicdomain/zero/1.0/legalcode.

/*
gcc view.c -o view -lSDL2 -lm $(sdl2-config --cflags --libs); ./simulation | ./view
*/

// Physics constants (from cerebras.c)
#define G 9.81       // Gravity (m/s²)
#define L1 1.0       // Length of first rod (m)
#define L2 1.5       // Length of second rod (m)
#define M1 1.0       // Mass of first rod (kg)
#define M2 1.5       // Mass of second rod (kg)

// Display constants
#define SCREEN_WIDTH 800
#define SCREEN_HEIGHT 600
#define WINDOW_TITLE "Humanoid Physics Visualization"
#define ROD_LENGTH_SCALE 100.0f  // Pixels per meter
#define MAX_DATA_LINES 1000      // Maximum number of data lines to store

// Structure to hold one line of simulation data
typedef struct {
    double prev_theta1;
    double prev_theta2;
    double start_theta1;
    double start_theta2;
    double end_theta1;
    double end_theta2;
    double tau1;
    double tau2;
} SimulationData;

// Array to store all simulation data
SimulationData data[MAX_DATA_LINES];
int data_count = 0;
int current_frame = 0;

// Graphics variables
SDL_Window* window = NULL;
SDL_Renderer* renderer = NULL;
int font_height = 16;
int paused = 0;
int step_mode = 0;
int playback_speed = 1; // Frames to advance per render

// Function to read simulation data from stdin
int read_simulation_data() {
    char line[256];
    int line_count = 0;
    
    // Skip header line
    if (fgets(line, sizeof(line), stdin) == NULL) {
        return 0;
    }
    
    // Read data lines
    while (fgets(line, sizeof(line), stdin) != NULL && line_count < MAX_DATA_LINES) {
        if (sscanf(line, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf",
                  &data[line_count].prev_theta1,
                  &data[line_count].prev_theta2,
                  &data[line_count].start_theta1,
                  &data[line_count].start_theta2,
                  &data[line_count].end_theta1,
                  &data[line_count].end_theta2,
                  &data[line_count].tau1,
                  &data[line_count].tau2) == 8) {
            line_count++;
        }
    }
    
    return line_count;
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
    
    return 1;
}

// Shutdown SDL
void shutdown_graphics() {
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
}

// Render the current frame of simulation
void render_simulation(int frame) {
    if (frame >= data_count) return;
    
    // Get current data
    SimulationData* current = &data[frame];
    
    // Clear screen
    SDL_SetRenderDrawColor(renderer, 30, 30, 30, 255); // Dark grey background
    SDL_RenderClear(renderer);

    // Define the origin point (fixed joint)
    int origin_x = SCREEN_WIDTH / 2;
    int origin_y = SCREEN_HEIGHT / 2;

    // Calculate positions based on angles (using end_theta values)
    int joint2_x = origin_x + (int)(ROD_LENGTH_SCALE * L1 * sin(current->end_theta1));
    int joint2_y = origin_y - (int)(ROD_LENGTH_SCALE * L1 * cos(current->end_theta1));
    
    int end_x = joint2_x + (int)(ROD_LENGTH_SCALE * L2 * sin(current->end_theta1 + current->end_theta2));
    int end_y = joint2_y - (int)(ROD_LENGTH_SCALE * L2 * cos(current->end_theta1 + current->end_theta2));

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
        (int)(bar_width * fabs(current->tau1) / 50.0), // Scale to reasonable size
        bar_height
    };
    
    if (current->tau1 >= 0) {
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
        (int)(bar_width * fabs(current->tau2) / 50.0), // Scale to reasonable size
        bar_height
    };
    
    if (current->tau2 >= 0) {
        SDL_SetRenderDrawColor(renderer, 50, 255, 50, 255); // Green for positive
        tau2_rect.x = joint2_x;
    } else {
        SDL_SetRenderDrawColor(renderer, 255, 50, 50, 255); // Red for negative
        tau2_rect.x = joint2_x - tau2_rect.w;
    }
    SDL_RenderFillRect(renderer, &tau2_rect);

    // Draw data text in top-left corner
    char data_text[512];
    sprintf(data_text, 
            "Frame: %d/%d\n"
            "Theta1: %.4f rad\n"
            "Theta2: %.4f rad\n"
            "Torque1: %.2f Nm\n"
            "Torque2: %.2f Nm\n"
            "\n"
            "Controls:\n"
            "Space: Pause/Resume\n"
            "Left/Right: Prev/Next Frame\n"
            "S: Step Mode Toggle\n"
            "+/-: Speed Up/Down\n"
            "R: Reset to Start\n"
            "Q/Esc: Quit",
            frame + 1, data_count,
            current->end_theta1, current->end_theta2,
            current->tau1, current->tau2);
    
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

int main() {
    // Read simulation data from stdin
    data_count = read_simulation_data();
    if (data_count == 0) {
        printf("No simulation data read from stdin. Exiting.\n");
        return 1;
    }
    
    printf("Read %d lines of simulation data.\n", data_count);
    
    // Initialize graphics
    if (!initialize_graphics()) {
        return 1;
    }
    
    // Event handling
    SDL_Event event;
    int running = 1;
    Uint32 last_time = SDL_GetTicks();
    Uint32 frame_delay = 50; // 20 fps by default
    
    // Main loop
    while (running) {
        // Handle events
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = 0;
            } else if (event.type == SDL_KEYDOWN) {
                switch (event.key.keysym.sym) {
                    case SDLK_ESCAPE:
                    case SDLK_q:
                        running = 0;
                        break;
                    case SDLK_SPACE:
                        paused = !paused;
                        break;
                    case SDLK_s:
                        step_mode = !step_mode;
                        if (step_mode) paused = 1;
                        break;
                    case SDLK_RIGHT:
                        if (step_mode || paused) {
                            current_frame = (current_frame + 1) % data_count;
                        } else {
                            current_frame = (current_frame + 10) % data_count;
                        }
                        break;
                    case SDLK_LEFT:
                        if (step_mode || paused) {
                            current_frame = (current_frame - 1 + data_count) % data_count;
                        } else {
                            current_frame = (current_frame - 10 + data_count) % data_count;
                        }
                        break;
                    case SDLK_r:
                        current_frame = 0;
                        break;
                    case SDLK_PLUS:
                    case SDLK_EQUALS:
                        playback_speed = playback_speed < 10 ? playback_speed + 1 : 10;
                        break;
                    case SDLK_MINUS:
                        playback_speed = playback_speed > 1 ? playback_speed - 1 : 1;
                        break;
                }
            }
        }
        
        // Update frame if not paused
        Uint32 current_time = SDL_GetTicks();
        if (!paused && current_time - last_time > frame_delay) {
            current_frame = (current_frame + playback_speed) % data_count;
            last_time = current_time;
        }
        
        // Render current frame
        render_simulation(current_frame);
    }
    
    // Cleanup
    shutdown_graphics();
    
    return 0;
}