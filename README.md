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
- Rocker (install via pip):

   ```bash
   pip install rocker
   ```

---

## ROS1 Docker Setup

### ROS1 Build and Run

1. **Build the Docker image:**

   ```bash
   docker build -t tiago_adn_ros_noetic -f ROS1_Docker/Dockerfile ./ROS1_Docker
   ```

2. **Run the Docker container:**

   ```bash
   rocker \
       --nvidia \
       --x11 \
       -- tiago_adn_ros_noetic
   ```

### ROS1 Development Workflow

When developing inside the container:

1. **Enter the container:**

   ```bash
   rocker \
       --nvidia \
       --x11 \
       -- tiago_adn_ros_noetic
   ```

2. **Once inside the container, you can open a terminal application:**

   ```bash
   terminator -u
   ```

---

## ROS2 Docker Setup

### ROS2 Build and Run

1. **Build the Docker image:**

   ```bash
   docker build -t tiago_adn_ros2_humble -f ROS2_Docker/Dockerfile ./ROS2_Docker
   ```

2. **Run the Docker container:**

   ```bash
   rocker \
       --nvidia \
       --cuda \
       --x11 \
       -- tiago_adn_ros2_humble
   ```

### ROS2 Development Workflow

When developing inside the container:

1. **Enter the container:**

   ```bash
   rocker \
       --nvidia \
       --cuda \
       --x11 \
       -- tiago_adn_ros2_humble
   ```

2. **Once inside the container, source the ROS2 workspace:**

   ```bash
   source /tiago_ws/install/setup.bash
   ```

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
