# Modular Construct Robot

## Requirements

- Ubuntu 22.04
- ROS 2 Humble

## Install

```bash
# git clone repo
mkdir -p ~/mdl_ws/src
cd ~/mdl_ws/src
git clone https://github.com/keigo06/mdl_robot.git
cd ~/mdl_ws && colcon build --symlink-install
```

- Install MoveIt Humble by following [Getting Started](https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html)

```bash
source /opt/ros/humble/setup.bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
sudo apt update
sudo apt dist-upgrade
sudo apt install python3-colcon-common-extensions
sudo apt install python3-colcon-mixin
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default

#Install vcstool
sudo apt install python3-vcstool

#Install MoveIt
sudo apt install ros-humble-moveit

sudo apt update
sudo apt install ros-humble-urdf-launch

source /opt/ros/humble/setup.bash
```

```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```
