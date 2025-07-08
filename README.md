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
- Rocker (for running Docker containers with GUI support)

### Getting Started

1. **Clone the TiaGO tutorials repository:**

   ```bash
   git clone https://github.com/pal-robotics/tiago_tutorials.git
   ```

2. **Navigate to the tiago_tutorials directory:**

   ```bash
   cd tiago_tutorials
   ```

3. **Build the Docker image:**

   ```bash
   docker build -t tiago_tutorials:original .
   ```

4. **Run the Docker container with rocker:**

   ```bash
   rocker --nvidia --x11 tiago_tutorials:original
   ```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.