<<<<<<< HEAD

# modular robot repo
=======
# Modular Construct Robot
>>>>>>> 8e2055b1b7d0686b0ef79b6e814b179ca5d104d4

## Requirements

- Ubuntu 22.04
- ROS 2 Humble

## Install

<<<<<<< HEAD
```
# Make a workspace
mkdir -p ~/manta_ws/src
cd ~/manta_ws/src
git clone https://github.com/tasada038/manta_v2.git
cd ~/manta_ws && colcon build --symlink-install
```
=======
```bash
# git clone repo
mkdir -p ~/mdl_ros2_ws/src
cd ~/mdl_ros2_ws/src
git clone https://github.com/keigo06/mdl_robot.git
cd ~/mdl_ros2_ws && colcon build --symlink-install
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
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-ros2-control*
sudo apt install ros-humble-moveit

sudo apt update
sudo apt install ros-humble-urdf-launch

source /opt/ros/humble/setup.bash
```

```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

## MoveIt Setup Assistant

I wrote this as a note for furure work reference

- Select Start Screen
  - Select Create New MoveIt Configuration Package
  - Select Browse
  - Select mdl_robot/mdl_description/urdf/mdl.xacro
- Select Self-Collision Panel
  - Generate Collision matrix
- Select Vitrual Joints Panel
  - Add Virtual Joint
    - Virtual Joint Name: virtual_joint_EE_1
    - Child Link: Link_1
    - Parent Frame Name: world
    - Joint Type: fixed
    TODO: Select Vitrual Joint
  - ここで根本と手先両方選ぶべきなのかわからない
  - Worldに固定したいときには追加する, とも
  - 台車＋アームのときには追加する, とも書かれている
  - とりあえずは選択しない
- Select Plannning Groups Panel
  - Add Group
    - Group Name: simple_5dof_robot
    - Kinematic Solver: kdl_kinematics_plugin/KDLKinematicsPlugin
    - Kine.~: default setting OK
    - Group Default Planner: BiTRRT
  - Add Joints
    - Select All Joints
- Select Robot Poses Panel
  - Pose Name: ready
  - Planning Group: simple_5dof_robot
  - Joint_1~5: 0.00
  - Save
- Select End Effectors Panel
  - TODO
- Select RIS 2 Controllers
  - Add Controller
    - Controller Name: simple_5dof_robot_controller
    - Controller Type: joint_trajectory_controller/JointTrajectoryController
    - Add Plannning Group Joints
      - Select simple_5dof_arm
      - Select >
      - Save
- Select MoveIt Controllers
  - Add Contorller
    - Controller Name: simple_5dof_robot_controller
    - Controller Type: FollowJointTrajectory
    - Action Namespace: follow_joint_trajetory
    - Add Plannning Group Joints
      - Select simple_5dof_arm
      - Select >
      - Save
- Perception
  - Pass
- Author Information
  - your Name and Email
- Configuration Files
  - /home/name/mdl_ros2_ws/src/mdl_robot/src/mdl_moveit_config
  - Generate Packages
  - Exit
>>>>>>> 8e2055b1b7d0686b0ef79b6e814b179ca5d104d4
