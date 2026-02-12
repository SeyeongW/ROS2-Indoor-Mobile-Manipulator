<h1 align = "center">waver_gazebo package</h1>

## Overview

The `waver_gazebo` package is designed to integrate the Wave Rover robot with the Gazebo simulation environment. This package includes the necessary launch files and configurations to simulate the Wave Rover in a realistic world. It allows to test and validate robot behaviors, algorithms, and interactions in a controlled and reproducible environment before deploying them on real hardware.

## Simulation

### Prerequisites
Before using the `waver_gazebo` package, ensure you have the following installed:
- Docker. [See for details](https://github.com/GGomezMorales/waver?tab=readme-ov-file#how-to-use).
- [waver_description package](https://github.com/GGomezMorales/waver/tree/noetic/waver_description).

### Simulation steps
1. Clone the repository:
   ```bash
   git clone https://github.com/GGomezMorales/waver.git
   cd waver
   ```

2. Build the Docker image:
   ```bash
   ./scripts/build.sh
   ```

3. Run the Docker container:
   There are three ways to run a container, depending on your host machine:
      - **`run_docker.sh`:** Uses the standard Docker command to run the container.
         ```bash
         ./scripts/run_docker.sh
         ```
      - **`run_cpu.sh`:** Uses [rocker](https://github.com/osrf/rocker) to run the container on a CPU environment.
         ```bash
         ./scripts/run_cpu.sh
         ```
      - **`run_nvidia.sh`:** Uses [rocker](https://github.com/osrf/rocker) to run the container with NVIDIA GPU support.
         ```bash
         ./scripts/run_nvidia.sh
         ```

   After running the container:
   ```bash
   root@10bb71e2357e:/waver_ws# 
   ```

4. ROS environment setup:
   The Docker image includes an alias to simplify the process of sourcing and building packages within a catkin workspace. To set up the ROS environment, use the following command:
   ```bash
   sros
   ```

5. Gazebo launch
   Finally, to launch the Gazebo simulation with the Wave Rover, use the following command:
   ```bash
   roslaunch waver_gazebo gazebo.launch
   ```
   To control the robot using teleoperation tools in another terminal use `./scripts/bash.sh` and then the following command:
   ```bash
   rosrun teleop_twist_keyboard teleop_twist_keyboard.py
   ```
