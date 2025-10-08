#!/bin/bash

# ResQoUnity One-Command Setup and Simulation Runner
# Copyright (c) 2024, RoboVerse community

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}=== ResQoUnity Simulation Setup ===${NC}"

# Set ROS_DISTRO
export ROS_DISTRO=humble

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$SCRIPT_DIR"

echo -e "${BLUE}Working from: $(pwd)${NC}"

# Activate conda orbit environment
echo -e "${YELLOW}Activating conda orbit environment...${NC}"
eval "$(conda shell.bash hook)"
conda activate orbit

# Setup ROS
echo -e "${BLUE}Setting up ROS environment...${NC}"
source /opt/ros/${ROS_DISTRO}/setup.bash

# Build ROS workspaces
if [[ -d "IsaacSim-ros_workspaces/${ROS_DISTRO}_ws" ]]; then
    echo -e "${YELLOW}Building Isaac Sim ROS workspace...${NC}"
    cd IsaacSim-ros_workspaces/${ROS_DISTRO}_ws
    rosdep install --from-paths src --ignore-src -r -y > /dev/null 2>&1 || true
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
    source install/setup.bash
    cd ../..
    echo -e "${GREEN}✓ Isaac Sim ROS workspace built${NC}"
fi

if [[ -d "go2_omniverse_ws" ]]; then
    echo -e "${YELLOW}Building Go2 workspace...${NC}"
    cd go2_omniverse_ws
    rosdep install --from-paths src --ignore-src -r -y > /dev/null 2>&1 || true
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
    source install/setup.bash
    cd ..
    echo -e "${GREEN}✓ Go2 workspace built${NC}"
fi

# Find IsaacLab
ISAACLAB_PATH="/home/kishi/IsaacLab_v0.3.1"
if [[ ! -d "$ISAACLAB_PATH" ]]; then
    ISAACLAB_PATH="$HOME/IsaacLab_v0.3.1"
fi

if [[ ! -d "$ISAACLAB_PATH" ]]; then
    echo -e "${RED}IsaacLab not found!${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Found IsaacLab at: $ISAACLAB_PATH${NC}"

# Setup IsaacLab
cd "$ISAACLAB_PATH"

# Install dependencies if needed
echo -e "${YELLOW}Checking IsaacLab setup...${NC}"

# First, install pydantic into Isaac Sim's Python (critical for extension loading)
echo -e "${YELLOW}Installing pydantic into Isaac Sim Python...${NC}"
"$ISAACLAB_PATH"/../isaac-sim*/python.sh -m pip install pydantic 2>/dev/null || \
    ~/.local/share/ov/pkg/isaac-sim*/python.sh -m pip install pydantic 2>/dev/null || \
    echo -e "${RED}Warning: Could not install pydantic into Isaac Sim Python${NC}"

# Install other dependencies into conda environment
echo -e "${YELLOW}Installing Python packages into conda environment...${NC}"
./orbit.sh -p -m pip install toml setuptools wheel build 2>/dev/null || true

# Try to install extensions if not already done
if ! ./orbit.sh -p -c "import omni.isaac.orbit; print('OK')" 2>/dev/null; then
    echo -e "${YELLOW}Installing IsaacLab extensions...${NC}"
    export SETUPTOOLS_USE_DISTUTILS=stdlib
    ./orbit.sh -p -m pip install -e source/extensions/omni.isaac.orbit --no-build-isolation 2>/dev/null || true
    ./orbit.sh -p -m pip install -e source/extensions/omni.isaac.orbit_tasks --no-build-isolation 2>/dev/null || true
    ./orbit.sh -p -m pip install -e source/extensions/omni.isaac.orbit_assets --no-build-isolation 2>/dev/null || true
fi

# Source ROS workspaces
source "$SCRIPT_DIR/IsaacSim-ros_workspaces/${ROS_DISTRO}_ws/install/setup.bash" 2>/dev/null || true
source "$SCRIPT_DIR/go2_omniverse_ws/install/setup.bash" 2>/dev/null || true

# Check GPU
echo -e "${BLUE}Checking GPU status...${NC}"
if command -v nvidia-smi &> /dev/null; then
    nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv
else
    echo -e "${YELLOW}nvidia-smi not found, GPU check skipped${NC}"
fi

# Run simulation with additional flags for better diagnostics
echo -e "${GREEN}=== Starting Simulation ===${NC}"
echo -e "${YELLOW}This may take a minute to initialize...${NC}"

# Run with Isaac Sim simulator (not just Python)
echo -e "${BLUE}Starting Isaac Sim with rendering...${NC}"
./orbit.sh --sim "$SCRIPT_DIR/main.py" --robot_amount 1 --robot go2 --terrain flat
