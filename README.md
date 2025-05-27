# Exoskeleton Motor Control Project

This project provides a framework for exoskeleton motor control, generating foundational model training data specifically designed for exoskeleton joints (elbows/ankles) that can be attached to humanoid toys from retailers like Home Depot or Lowe's. The system simulates robotic leg dynamics with physics-based models and provides visualization tools for torque control optimization.

What is an exoskeleton? Ask [crowsays.theme25.com](https://crowsays.theme25.com)

## Project Components

### Visualization Tools
- **view.c**: A visualization tool that uses SDL2 to render a blueprint-style schematic of a robotic leg exoskeleton. It reads simulation data from stdin, displaying leg segments (thigh, shin, foot) with mechanical details, joint articulations, and torque indicators. Includes interactive controls for playback, stepping through frames, and adjusting simulation speed.

### Simulation Engines
- **standalone.c**: A self-contained physics simulation and visualization program that combines the simulation logic with real-time rendering. It models a two-segment robotic leg with gravitational forces and PD control, applying random noise to simulate real-world conditions.

- **simulation.c**: Generates pure simulation data for the exoskeleton leg model using physics equations. It calculates gravitational torques and applies PD control with noise to produce realistic motion patterns. Outputs a dataset with angular positions and torques that can be piped to the visualization program or used for training ML models.

### Control Systems
- **robot-control-local.py**: A local version of the control system that doesn't require external API calls. Uses Euclidean distance calculations to find the closest matching angle configurations in the dataset and applies their torque values to control the exoskeleton.

- **robot-control-openrouter.py**: Implements a control system using OpenRouter API to find optimal torques for given angles by matching against the reference dataset. Simulates the exoskeleton's motion while leveraging AI capabilities to determine the best control parameters.

- **robot-control-cerebras.py**: An advanced implementation that uses Cerebras AI platform to match angle configurations with optimal torques. Provides more sophisticated pattern matching capabilities than the local version while maintaining the core physics simulation components.

### Testing and Data
- **robot-unit-test.py**: A testing utility that validates the system's ability to find matching torque values for given theta (angle) inputs. It communicates with OpenRouter API to process the test data, comparing the results against expected values.

- **robot-test.txt**: Contains test data for the exoskeleton control system with multiple lines of simulation data (angles and torques). Used by the testing utilities to validate the model's ability to match angles with appropriate torque values.

- **robot-control.txt**: A dataset of reference values containing sequences of angles and their corresponding optimal torque values. Serves as a lookup table for the control system to determine appropriate torques based on matching similar angle configurations.

## Getting Started

### Building the Simulation and Visualization Tools
```bash
# Compile the visualization tool
gcc view.c -o view -lSDL2 -lm $(sdl2-config --cflags --libs)

# Compile the simulation generator
gcc simulation.c -o simulation -lSDL2 -lm $(sdl2-config --cflags --libs)

# Compile the standalone simulator
gcc standalone.c -o standalone -lSDL2 -lm $(sdl2-config --cflags --libs)
```

### Running the Simulation with Visualization
```bash
# Generate simulation data and pipe it to the visualization tool
./simulation | ./view

# Run the standalone simulator
./standalone
```

## License
This project is licensed under Creative Commons CC0. To the extent possible under law, the author(s) have dedicated all copyright and related and neighboring rights to this document to the public domain worldwide. This document is distributed without any warranty.

[https://x.com/szegedi_mi54955/status/1926427056226632166](https://x.com/szegedi_mi54955/status/1926427056226632166)
