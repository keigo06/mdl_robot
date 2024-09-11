
# modular robot repo

## Requirements

- Ubuntu 22.04
- ROS 2 Humble

## Install

```
# Make a workspace
mkdir -p ~/manta_ws/src
cd ~/manta_ws/src
git clone https://github.com/tasada038/manta_v2.git
cd ~/manta_ws && colcon build --symlink-install
```