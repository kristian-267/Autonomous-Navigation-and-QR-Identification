# Autonomous Navigation and QR Identification

## Overview
This project focuses on autonomous navigation and QR code identification, leveraging ROS (Robot Operating System) for robotics and automation tasks. It includes scripts and resources for simulating a robot's movement and QR code recognition capabilities.

## Features
- Detailed robot models and configurations.
- Custom simulation environments in ROS.
- Autonomous navigation algorithms.
- QR code identification and processing.

## Files
- `launch\`: including files used to initialize and configurate simulation environment in ROS.
- `maps\`: including map files created from SLAM mapping the simulation environment.
- `models\`: including model configuration files used in the simulation environment.
- `scripts\navigation_and_scan.py`: main python script for the project.
- `scripts\spawn_barriers.py`: randomly spawn obstacles in the simulation environment.
- `scripts\spawn_markers.py`: randomly spawn QR markers in the simulation environment.
- `urdf\`: including files used to define the physical properties and appearances of markers, obstacles, and the robot in Gazebo.
- `worlds\`: including a world file used to define the properties of the simulated world in Gazebo.

## Requirements
- Ubuntu 18.04
- ROS Melodic, refer to: http://wiki.ros.org/melodic/Installation/Ubuntu
- Python 3.10

## Installation
Clone the repository and install dependencies listed in `requirements.txt`.

## Usage
Refer to the 'scripts' directory for execution instructions. Use 'launch' files to initiate simulations.
