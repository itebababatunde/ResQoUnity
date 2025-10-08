#!/bin/bash

# Simple working script - back to basics

export ROS_DISTRO=humble

# Setup ROS
source /opt/ros/$ROS_DISTRO/setup.bash

# Build workspaces
cd ~/ResQoUnity/IsaacSim-ros_workspaces/${ROS_DISTRO}_ws
colcon build
source install/setup.bash

cd ~/ResQoUnity/go2_omniverse_ws
colcon build
source install/setup.bash

# Source workspaces
source ~/ResQoUnity/IsaacSim-ros_workspaces/${ROS_DISTRO}_ws/install/setup.bash
source ~/ResQoUnity/go2_omniverse_ws/install/setup.bash

# Run with IsaacLab (DON'T install pydantic - it breaks things!)
cd ~/IsaacLab_v0.3.1
./orbit.sh --sim ~/ResQoUnity/main.py --robot_amount 1 --robot go2 --terrain flat

