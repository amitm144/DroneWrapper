# Quadcopter Simulation Wrapper

This wrapper was created for personal use and self-learning about the fascinating world of drones and flight dynamics. It was built for my custom purposes, tailored specifically to my learning needs and experimental requirements.

Special thanks to the creators of the original [Quadcopter_SimCon](https://github.com/bobzwik/Quadcopter_SimCon) simulation repository, which provided an excellent foundation for learning about drone flight mechanics and control systems.

## Overview

The wrapper enables users to:
1. Define waypoints for the drone to follow
2. Visualize the drone's flight path in real-time 3D animation
3. See motor thrust levels during flight
4. Navigate from one waypoint to the next

## Requirements

- NumPy
- Matplotlib
- The Quadcopter_SimCon simulation package (must be in a subdirectory)

## Installation

1. Make sure you have the required dependencies installed:
   ```
   pip install numpy matplotlib
   ```

2. Place this script in the parent directory of your Quadcopter_SimCon simulation folder.

## Usage

Run the script from the command line:
```
python wrapper.py
```

The program will ask for X, Y, Z coordinates, run the simulation, then prompt for the next target.

### Coordinate System

The simulation uses the NED (North-East-Down) coordinate system:
- X: North, Y: East, Z: Down (negative values indicate altitude)
- Z=2 means 2 meters up from the ground

## Features

- **Trajectory Visualization**: See the planned path
- **Motor Thrust Vector Display**: Shows thrust percentage for each motor
- **Real-time Position Tracking**: Displays current coordinates
- **Detailed 3D Visualization**: View from any angle

## Customization

Modifiable parameters: Flight time (Tf), Average velocity (v_average), Simulation timestep (Ts), Animation speed (numFrames)
