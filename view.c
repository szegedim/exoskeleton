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

#define LEG_WIDTH 20        // Width of the leg segments
#define KNEE_RADIUS 8       // Radius of the knee joint
#define HIP_RADIUS 10       // Radius of the hip joint
#define FOOT_LENGTH 40      // Length of the foot
#define FOOT_HEIGHT 10      // Height of the foot

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

    // Define the origin point (hip joint)
    int origin_x = SCREEN_WIDTH / 2;
    int origin_y = SCREEN_HEIGHT / 2;

    // Calculate joint positions based on angles
    int knee_x = origin_x + (int)(ROD_LENGTH_SCALE * L1 * sin(current->end_theta1));
    int knee_y = origin_y - (int)(ROD_LENGTH_SCALE * L1 * cos(current->end_theta1));
    
    int ankle_x = knee_x + (int)(ROD_LENGTH_SCALE * L2 * sin(current->end_theta1 + current->end_theta2));
    int ankle_y = knee_y - (int)(ROD_LENGTH_SCALE * L2 * cos(current->end_theta1 + current->end_theta2));

    // Calculate angles for drawing the leg segments
    float thigh_angle = current->end_theta1;
    float shin_angle = current->end_theta1 + current->end_theta2;
    
    // Calculate the vertices for the thigh (upper leg)
    SDL_Point thigh_points[4];
    // Half width perpendicular to the thigh angle
    int half_width = LEG_WIDTH / 2;
    
    // Calculate perpendicular offsets to the thigh line
    int dx_perp = (int)(half_width * cos(thigh_angle));
    int dy_perp = (int)(half_width * sin(thigh_angle));
    
    // Four corners of the thigh rectangle
    thigh_points[0].x = origin_x - dx_perp;
    thigh_points[0].y = origin_y + dy_perp;
    
    thigh_points[1].x = origin_x + dx_perp;
    thigh_points[1].y = origin_y - dy_perp;
    
    thigh_points[2].x = knee_x + dx_perp;
    thigh_points[2].y = knee_y - dy_perp;
    
    thigh_points[3].x = knee_x - dx_perp;
    thigh_points[3].y = knee_y + dy_perp;
    
    // Calculate the vertices for the shin (lower leg)
    SDL_Point shin_points[4];
    
    // Calculate perpendicular offsets to the shin line
    dx_perp = (int)(half_width * cos(shin_angle));
    dy_perp = (int)(half_width * sin(shin_angle));
    
    // Four corners of the shin rectangle
    shin_points[0].x = knee_x - dx_perp;
    shin_points[0].y = knee_y + dy_perp;
    
    shin_points[1].x = knee_x + dx_perp;
    shin_points[1].y = knee_y - dy_perp;
    
    shin_points[2].x = ankle_x + dx_perp;
    shin_points[2].y = ankle_y - dy_perp;
    
    shin_points[3].x = ankle_x - dx_perp;
    shin_points[3].y = ankle_y + dy_perp;
    
    // Calculate foot points
    SDL_Point foot_points[4];
    float foot_angle = shin_angle - M_PI/2; // Perpendicular to shin
    
    int foot_dx = (int)(FOOT_LENGTH * cos(foot_angle));
    int foot_dy = (int)(FOOT_LENGTH * sin(foot_angle));
    
    foot_points[0].x = ankle_x - (int)(half_width * cos(shin_angle));
    foot_points[0].y = ankle_y + (int)(half_width * sin(shin_angle));
    
    foot_points[1].x = ankle_x + (int)(half_width * cos(shin_angle));
    foot_points[1].y = ankle_y - (int)(half_width * sin(shin_angle));
    
    foot_points[2].x = foot_points[1].x + foot_dx;
    foot_points[2].y = foot_points[1].y + foot_dy;
    
    foot_points[3].x = foot_points[0].x + foot_dx;
    foot_points[3].y = foot_points[0].y + foot_dy;
    
    // Draw blueprint background grid
    SDL_SetRenderDrawColor(renderer, 50, 50, 80, 255); // Dark blue-ish grid
    
    // Draw vertical grid lines
    for (int x = 0; x < SCREEN_WIDTH; x += 50) {
        SDL_RenderDrawLine(renderer, x, 0, x, SCREEN_HEIGHT);
    }
    
    // Draw horizontal grid lines
    for (int y = 0; y < SCREEN_HEIGHT; y += 50) {
        SDL_RenderDrawLine(renderer, 0, y, SCREEN_WIDTH, y);
    }
    
    // Draw thigh outline (blueprint style)
    SDL_SetRenderDrawColor(renderer, 100, 180, 255, 255); // Light blue for blueprint
    
    // Draw thigh rectangle
    SDL_RenderDrawLine(renderer, thigh_points[0].x, thigh_points[0].y, thigh_points[1].x, thigh_points[1].y);
    SDL_RenderDrawLine(renderer, thigh_points[1].x, thigh_points[1].y, thigh_points[2].x, thigh_points[2].y);
    SDL_RenderDrawLine(renderer, thigh_points[2].x, thigh_points[2].y, thigh_points[3].x, thigh_points[3].y);
    SDL_RenderDrawLine(renderer, thigh_points[3].x, thigh_points[3].y, thigh_points[0].x, thigh_points[0].y);
    
    // Draw internal structure lines for thigh (mechanical details)
    int mid_thigh_x = (origin_x + knee_x) / 2;
    int mid_thigh_y = (origin_y + knee_y) / 2;
    SDL_RenderDrawLine(renderer, mid_thigh_x - dx_perp, mid_thigh_y + dy_perp, mid_thigh_x + dx_perp, mid_thigh_y - dy_perp);
    
    // Draw shin outline
    SDL_RenderDrawLine(renderer, shin_points[0].x, shin_points[0].y, shin_points[1].x, shin_points[1].y);
    SDL_RenderDrawLine(renderer, shin_points[1].x, shin_points[1].y, shin_points[2].x, shin_points[2].y);
    SDL_RenderDrawLine(renderer, shin_points[2].x, shin_points[2].y, shin_points[3].x, shin_points[3].y);
    SDL_RenderDrawLine(renderer, shin_points[3].x, shin_points[3].y, shin_points[0].x, shin_points[0].y);
    
    // Draw internal structure lines for shin (mechanical details)
    int mid_shin_x = (knee_x + ankle_x) / 2;
    int mid_shin_y = (knee_y + ankle_y) / 2;
    SDL_RenderDrawLine(renderer, mid_shin_x - dx_perp, mid_shin_y + dy_perp, mid_shin_x + dx_perp, mid_shin_y - dy_perp);
    
    // Draw foot
    SDL_RenderDrawLine(renderer, foot_points[0].x, foot_points[0].y, foot_points[1].x, foot_points[1].y);
    SDL_RenderDrawLine(renderer, foot_points[1].x, foot_points[1].y, foot_points[2].x, foot_points[2].y);
    SDL_RenderDrawLine(renderer, foot_points[2].x, foot_points[2].y, foot_points[3].x, foot_points[3].y);
    SDL_RenderDrawLine(renderer, foot_points[3].x, foot_points[3].y, foot_points[0].x, foot_points[0].y);
    
    // Draw foot internal structure (arch support)
    int mid_foot_x = (foot_points[0].x + foot_points[2].x) / 2;
    int mid_foot_y = (foot_points[0].y + foot_points[2].y) / 2;
    SDL_RenderDrawLine(renderer, ankle_x, ankle_y, mid_foot_x, mid_foot_y);
    
    // Draw hip joint (circle)
    SDL_SetRenderDrawColor(renderer, 200, 220, 255, 255); // Lighter blue for joints
    for (int i = -HIP_RADIUS; i <= HIP_RADIUS; i++) {
        for (int j = -HIP_RADIUS; j <= HIP_RADIUS; j++) {
            if (i*i + j*j <= HIP_RADIUS*HIP_RADIUS && i*i + j*j >= (HIP_RADIUS-2)*(HIP_RADIUS-2)) {
                SDL_RenderDrawPoint(renderer, origin_x + i, origin_y + j);
            }
        }
    }
    
    // Draw knee joint (circle with mechanical details)
    for (int i = -KNEE_RADIUS; i <= KNEE_RADIUS; i++) {
        for (int j = -KNEE_RADIUS; j <= KNEE_RADIUS; j++) {
            if (i*i + j*j <= KNEE_RADIUS*KNEE_RADIUS && i*i + j*j >= (KNEE_RADIUS-2)*(KNEE_RADIUS-2)) {
                SDL_RenderDrawPoint(renderer, knee_x + i, knee_y + j);
            }
        }
    }
    
    // Draw crosshairs in the knee joint
    SDL_RenderDrawLine(renderer, knee_x - KNEE_RADIUS, knee_y, knee_x + KNEE_RADIUS, knee_y);
    SDL_RenderDrawLine(renderer, knee_x, knee_y - KNEE_RADIUS, knee_x, knee_y + KNEE_RADIUS);
    
    // Draw ankle joint
    for (int i = -KNEE_RADIUS/1.5; i <= KNEE_RADIUS/1.5; i++) {
        for (int j = -KNEE_RADIUS/1.5; j <= KNEE_RADIUS/1.5; j++) {
            if (i*i + j*j <= (KNEE_RADIUS/1.5)*(KNEE_RADIUS/1.5) && i*i + j*j >= (KNEE_RADIUS/1.5-2)*(KNEE_RADIUS/1.5-2)) {
                SDL_RenderDrawPoint(renderer, ankle_x + i, ankle_y + j);
            }
        }
    }

    // Display torque indicators
    int bar_width = 50;
    int bar_height = 10;
    
    // Torque 1 indicator (hip joint)
    SDL_Rect tau1_rect = {
        origin_x - bar_width/2,
        origin_y + 20,
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
    
    // Torque 2 indicator (knee joint)
    SDL_Rect tau2_rect = {
        knee_x - bar_width/2,
        knee_y + 20,
        (int)(bar_width * fabs(current->tau2) / 50.0), // Scale to reasonable size
        bar_height
    };
    
    if (current->tau2 >= 0) {
        SDL_SetRenderDrawColor(renderer, 50, 255, 50, 255); // Green for positive
        tau2_rect.x = knee_x;
    } else {
        SDL_SetRenderDrawColor(renderer, 255, 50, 50, 255); // Red for negative
        tau2_rect.x = knee_x - tau2_rect.w;
    }
    SDL_RenderFillRect(renderer, &tau2_rect);

    // Add blueprint labels
    SDL_SetRenderDrawColor(renderer, 200, 220, 255, 255); // Light blue for text
    
    // Label for hip joint
    char hip_label[] = "HIP JOINT";
    int label_x = origin_x - 40;
    int label_y = origin_y - 25;
    for (size_t i = 0; i < strlen(hip_label); i++) {
        SDL_RenderDrawLine(renderer, label_x + i*8, label_y, label_x + i*8 + 5, label_y);
    }
    
    // Label for knee joint
    char knee_label[] = "KNEE JOINT";
    label_x = knee_x - 45;
    label_y = knee_y - 25;
    for (size_t i = 0; i < strlen(knee_label); i++) {
        SDL_RenderDrawLine(renderer, label_x + i*8, label_y, label_x + i*8 + 5, label_y);
    }
    
    // Label for ankle joint
    char ankle_label[] = "ANKLE";
    label_x = ankle_x - 25;
    label_y = ankle_y - 20;
    for (size_t i = 0; i < strlen(ankle_label); i++) {
        SDL_RenderDrawLine(renderer, label_x + i*8, label_y, label_x + i*8 + 5, label_y);
    }

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