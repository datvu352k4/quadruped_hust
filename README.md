# QUADRUPED_HUST 

**QUADRUPED_HUST** is a robotics project focused on applying **Reinforcement Learning (RL)** to train the **Unitree Go2** Quadruped robot. It enables the robot to learn locomotion based on velocity commands while maintaining seamless communication with the **ROS2** ecosystem.

---

## 🌟 Key Features

* **Reinforcement Learning:** Trains the robot using **PPO** within a custom environment.
* **ROS2 Integration:** Full communication bridge with ROS2 for sensor data and command handling.
* **Teleoperation:** Real-time robot control using a Joystick.
* **SLAM (Mapping):** Map construction capabilities using `slam-toolbox`.
* **Navigation:** Autonomous path planning and trajectory execution using `Nav2`.

---

## ⚙️ System Requirements

Ensure your system meets the following compatibility requirements:

* **OS:** Ubuntu 22.04
* **ROS Version:** ROS2 Humble
* **Simulator:** Genesis v0.3.5
* **Python:** 3.10+

---

## 🛠️ Installation

Follow these steps to set up the development environment.

### 1. Install Python Dependencies
Install the required Reinforcement Learning libraries and the specific NumPy version needed for compatibility.

```
pip install tensorboard rsl-rl-lib==2.2.4
pip install numpy==1.26.4
```

### 2. Install Genesis Simulator

Install Genesis v0.3.5 following the official instructions from their repository:

https://github.com/Genesis-Embodied-AI/Genesis?tab=readme-ov-file 

### 3. Install ROS2 Packages (Required)

Install the necessary ROS2 packages for Navigation, SLAM, and Joystick control.
Bash
```
sudo apt update
sudo apt install ros-humble-navigation2 \
ros-humble-nav2-bringup \
ros-humble-slam-toolbox \
ros-humble-teleop-twist-joy \
ros-humble-joy \
ros-humble-xacro
```

### 4. Setup Workspace & Build

Clone this repository into your ROS2 workspace (e.g., ~/ros2_ws) and build the project.

# Navigate to your workspace src folder
```
cd ~/ros2_ws/src
```
# Clone the repository 

# Install dependencies using rosdep
```
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

# Build the workspace
```
colcon build
```

## 🚀 Usage
### 1. Training Mode 

To start training the robot using Reinforcement Learning:
```
python3 go2_train.py
```

Note: You can adjust environment and training hyperparameters directly in the train and env configuration files.

### 2. Control Mode (Joystick) 🎮

To control the robot manually using a joystick:    
```
install/setup.bash
```

Launch the simulation bridge:
```
ros2 launch quadruped_bringup go2_sim.launch.py
```
Ensure your joystick is connected.
Joystick parameters can be modified in joystick.yaml.

### 3. SLAM Mode (Mapping) 🗺️

To run Simultaneous Localization and Mapping:
```
ros2 launch quadruped_bringup go2_sim.launch.py
```
Launch SLAM Toolbox:
```
ros2 launch quadruped_bringup go2_slam.launch.py
```
Save the Map: Once you have scanned the area, save the map using the service call:
```
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'my_map_name'}}"
```

### 4. Navigation Mode

To perform autonomous navigation on a saved map:
```
ros2 launch quadruped_bringup go2_sim.launch.py
```

Launch Nav2: Replace 'Path/to/your/map.yaml' with the actual absolute path to your saved map file.
```
ros2 launch quadruped_bringup go2_nav2.launch.py map:='/home/user/map_name.yaml'
```

### Acknowledgments

This project is built upon and inspired by the following repository:

https://github.com/vybhav-ibr/genesis_ros



