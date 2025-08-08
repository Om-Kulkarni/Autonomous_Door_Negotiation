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



## Connecting to the Robot

To interact with the robot, follow these steps:

### 1. Connect to the Robot's WiFi

- Join the `tiago-0c` WiFi access point.

### 2. Access the Web Interface (webCommander)

- Open your browser and navigate to: [http://10.68.0.1:8080/](http://10.68.0.1:8080/)

### 3. SSH into the Robot

Use the following command to connect via SSH:

```bash
ssh -oHostKeyAlgorithms=+ssh-rsa root@10.68.0.1
```

> **Note:** The `-oHostKeyAlgorithms=+ssh-rsa` option is required for compatibility with the robot's SSH server.



## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.


## ARM topics

```bash
/arm_controller/command
/arm_controller/follow_joint_trajectory/cancel
/arm_controller/follow_joint_trajectory/feedback
/arm_controller/follow_joint_trajectory/goal
/arm_controller/follow_joint_trajectory/result
/arm_controller/follow_joint_trajectory/status
/arm_controller/safe_command
/arm_controller/state
/arm_current_limit_controller/command
/arm_current_limit_controller/state

```

## SAMPLE:

/home/pal/tutorials_ws/src/

## EXAMPLES

![Joint Limit](Images/jointlimits.png)
ARM CONTROL(REFER LIMITS FROM WEBPAGE)
```bash
rostopic pub /arm_controller/command trajectory_msgJointTrajectory "header:
  seq: 26
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
  - 'arm_1_joint'
  - 'arm_2_joint'
  - 'arm_3_joint'
  - 'arm_4_joint'
  - 'arm_5_joint'
  - 'arm_6_joint'
  - 'arm_7_joint'
points:
- positions: [1, -0.7457, -2.9648, 1.7901, -2.0943, -0.5314, -0.1771]
  velocities: []
  accelerations: []
  effort: []
  time_from_start:
    secs: 1
    nsecs: 8481387"
```

TORSO CONTROL(REFER LIMITS FROM WEBPAGE)
```bash
 rostopic pub /torso_controller/command  trajectory_gs/JointTrajectory "header:
  seq: 26
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
  - 'torso_lift_joint'
points:
- positions: [0.1]
  velocities: []
  accelerations: []
  effort: []
  time_from_start:
    secs: 1
    nsecs: 1"

```
HEAD CONTROL(REFER LIMITS FROM WEBPAGE)

```bash
rostopic pub /head_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0 
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
  - 'head_1_joint'
  - 'head_2_joint'
points:          
- positions: [1.0,-0.6]
  velocities: []
  accelerations: []
  effort: []
  time_from_start:
    secs: 1
    nsecs: 8481387"
```

GRIPPER CONTROL


gripper limits:
 
Maximum:
   
positions: [0.053180563895850796, 0.05262160242463829]

Minimum:

positions: [-0.008305216756601662, -0.008124320836811796]

```bash
rostopic pub /gripper_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
  - 'gripper_right_finger_joint'
  - 'gripper_left_finger_joint'
points:
- positions: [0.05,0.05]
  velocities: []
  accelerations: []
  effort: []
  time_from_start:
    secs: 1
    nsecs: 8481387"
```

BASE CONTROL:

Topics to control the base

Rotate around Z axis (set speed to -0.3 or 0.3  for clockwise and anticlockwise )
Moving forward change the linear x(set speed 0.2 or -0.2 to move forward or backward)

```bash
rostopic pub -r 15 /mobile_base_controller/cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -0.3" 
```

