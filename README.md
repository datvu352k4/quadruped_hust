Markdown

# QUADRUPED_HUST: RL & ROS 2 Integration for Go2

[![Ubuntu](https://img.shields.io/badge/OS-Ubuntu_22.04-orange.svg)](https://releases.ubuntu.com/22.04/)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-green.svg)](https://docs.ros.org/en/humble/index.html)
[![Genesis](https://img.shields.io/badge/Genesis-v0.3.5-blue.svg)](https://github.com/Genesis-Embodied-AI/Genesis)
[![Python](https://img.shields.io/badge/Python-3.10-yellow.svg)](https://www.python.org/)

**QUADRUPED_HUST** is a robotics project focused on integrating **Reinforcement Learning (RL)** with **ROS 2** for the Unitree Go2 Quadruped robot. The robot is trained to follow velocity commands using PPO in a custom environment and can perform SLAM and autonomous navigation.

---

## 🚀 Key Features

* **Reinforcement Learning:** Trains the robot using PPO (Proximal Policy Optimization) with custom environments.
* **ROS 2 Bridge:** Seamless communication between the RL agent and the ROS 2 ecosystem.
* **Joystick Control:** Real-time manual control support.
* **SLAM:** Environment mapping using `slam_toolbox`.
* **Autonomous Navigation:** Path planning and trajectory execution using `Nav2`.

---

## 🛠️ System Requirements

* **Operating System:** Ubuntu 22.04 LTS
* **ROS Distribution:** ROS 2 Humble Hawksbill
* **Simulator:** Genesis v0.3.5
* **Python:** 3.10+

---

## 📦 Installation

### 1. Install ROS 2 Dependencies
Ensure you have ROS 2 Humble installed. Then, install the necessary navigation, control, and utility packages:


sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup 
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-joy ros-humble-teleop-twist-joy
sudo apt install ros-humble-xacro

### 2. Install Python Dependencies

Install the required Reinforcement Learning libraries and specific Numpy version:
Bash

pip install tensorboard rsl-rl-lib==2.2.4
pip install numpy==1.26.4

### 3. Install Genesis Simulator

Follow the official instructions to install Genesis v0.3.5:

    Genesis Installation Guide

### 4. Setup Workspace

Clone this repository and build the workspace:
Bash

# 1. Create directory (if not exists) and enter src
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src

# 2. Clone this repository
git clone [https://github.com/datvu352k4/quadruped_hust.git](https://github.com/datvu352k4/quadruped_hust.git)

# 3. Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 4. Build the workspace
colcon build --symlink-install

## 🎮 Usage Guide
### 1. Training Mode (Reinforcement Learning)

To train the robot using the PPO algorithm:

# Run the training script
python3 src/quadruped_hust/scripts/go2_train.py

    Note: You can adjust the environment and training hyperparameters in the train and env configuration files.

### 2. Manual Control (Joystick)

To control the robot using a gamepad/joystick:
Bash

source install/setup.bash

# Launch simulation and control node
ros2 launch quadruped_bringup go2_sim.launch.py

    Ensure your joystick is connected.

    Configuration: Modify parameters in config/joystick.yaml.

### 3. SLAM (Mapping)

To generate a map of the environment:

Step 1: Launch the simulation
Bash

ros2 launch quadruped_bringup go2_sim.launch.py

Step 2: Launch SLAM Toolbox
Bash

ros2 launch quadruped_bringup go2_slam.launch.py

Step 3: Save the map Once you have scanned the area, run this command to save the map:
Bash

ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/your_user/ros2_ws/maps/my_map_name'}}"

### 4. Navigation (Nav2)

To perform autonomous navigation on a saved map:
Bash

# Terminal 1: Simulation
ros2 launch quadruped_bringup go2_sim.launch.py

# Terminal 2: Navigation
ros2 launch quadruped_bringup go2_nav2.launch.py map:='/path/to/your/map.yaml'

🙌 Acknowledgements

This project is built upon the foundation of genesis_ros. Special thanks to the authors for their contributions.

