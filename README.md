# Autonomous Door Negotiation

A research project focused on developing autonomous robotic systems capable of effectively negotiating doors in various real-world environments. This project explores the challenges and solutions in door detection, handle manipulation, and successful door traversal for mobile robots.

## Overview

- Door detection and classification
- Handle manipulation strategies
- Autonomous navigation through doorways
- Safety considerations for robot-door interaction

## Setup and Installation

### Prerequisites

- Docker installed on your system
- Git
- Rocker (for running Docker containers with GUI support) - Follow the [installation guide](https://github.com/osrf/rocker)

---

## ROS1 Docker Setup

### Getting Started

1. **Build the Docker image:**

   ```bash
   docker build -t tiago_adn_ros_noetic -f ROS1_Docker/Dockerfile .
   ```

2. **Run the Docker container with volume mounting (for development):**

   ```bash
   rocker --nvidia --x11 --volume "$(pwd)/ROS1_Docker/tiago_adn:/tiago_public_ws/src/tiago_adn" -- tiago_adn_ros_noetic
   ```

   This mounts your local `tiago_adn` folder to the container so any changes made inside the container will persist on your host machine.

3. **Alternative: Run without volume mounting:**

   ```bash
   rocker --nvidia --x11 -- tiago_adn_ros_noetic
   ```

### Development Workflow

When developing inside the container with volume mounting:

1. **Enter the container:**

   ```bash
   rocker --nvidia --x11 --volume "$(pwd)/ROS1_Docker/tiago_adn:/tiago_public_ws/src/tiago_adn" -- tiago_adn_ros_noetic
   ```

2. **Once inside the container, you can open a terminal application:**

   ```bash
   terminator -u
   ```

---

## ROS2 Docker Setup

### Getting Started

1. **Build the Docker image:**

   ```bash
   docker build -t tiago_adn_ros2_humble -f ROS2_Docker/Dockerfile .
   ```

2. **Run the Docker container with GPU support:**

   ```bash
   docker run --gpus all -it --rm \
       -v "$(pwd):/home/om-kulkarni/Projects/Autonomous_Door_Negotiation" \
       -w /tiago_ws \
       tiago_adn_ros2_humble
   ```

   This mounts your project directory into the container and sets the working directory to the ROS2 workspace.

### Development Workflow

When developing inside the container:

1. **Enter the container:**

   ```bash
   docker run --gpus all -it --rm \
       -v "$(pwd):/home/om-kulkarni/Projects/Autonomous_Door_Negotiation" \
       -w /tiago_ws \
       tiago_adn_ros2_humble
   ```

2. **Once inside the container, source the ROS2 workspace:**

   ```bash
   source /tiago_ws/install/setup.bash
   ```

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.