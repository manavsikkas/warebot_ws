# Warebot Workspace

ROS 2 workspace for the Warebot robot platform with Docker support and development environment configuration.

## Overview

This repository contains the ROS 2 package for the Warebot robot, including URDF descriptions, 3D meshes, launch files, and development environment setup.

## Project Structure

```
warebot_ws/
├── .devcontainer/          # VS Code dev container configuration
├── .gitignore              # Git ignore rules for build artifacts
├── Dockerfile              # Docker setup for ROS 2 development
├── start.sh               # Script to run Docker container with GUI support
├── src/
│   └── warebot/           # Main ROS 2 package
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── description/    # Robot URDF descriptions and meshes
│       │   ├── meshes/     # 3D model files (COLLADA .dae and STL)
│       │   └── urdf/       # URDF/Xacro files
│       └── launch/         # ROS 2 launch files
└── README.md
```

## Features

- **Docker Support**: Complete Docker setup for ROS 2 development environment
- **VS Code Integration**: Dev container configuration for seamless development
- **3D Models**: Comprehensive mesh files for robot components including:
  - Base link and top chassis
  - Lidar mounts and sensor arches
  - Bumper extensions
  - Wheel models (indoor/outdoor)
- **URDF Descriptions**: Modular robot description files using Xacro
- **Launch Files**: Ready-to-use launch files for visualization and testing

## Prerequisites

- Docker
- ROS 2 (Jazzy or compatible)
- VS Code (optional, for dev container support)

## Getting Started

### Using Docker

1. Build the Docker image:
   ```bash
   docker build -t warebot-jazzy .
   ```

2. Run the container with GUI support:
   ```bash
   ./start.sh
   ```

### Local Development

1. Clone the repository:
   ```bash
   git clone https://github.com/manavsikkas/warebot_ws.git
   cd warebot_ws
   ```

2. Build the workspace:
   ```bash
   colcon build
   ```

3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

4. Launch the robot visualization:
   ```bash
   ros2 launch warebot display.launch.py
   ```

## Development Environment

The project includes:
- `.gitignore` file to exclude build artifacts and IDE settings
- Dockerfile for setting up ROS 2 development environment with necessary packages
- `start.sh` script for running the Docker container with GUI support
- `.devcontainer/devcontainer.json` for VS Code integration with Docker

## Robot Description

The robot description includes:
- Base platform (A200)
- Modular attachment system
- Sensor mounting points
- Wheel configurations (indoor/outdoor)

## Launch Files

- `display.launch.py`: Launches RViz2 with robot visualization and joint state publisher GUI

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## License

[Add your license here]

## Author

Manav Sikkas

