#!/bin/bash

# Copyright (c) 2024, RoboVerse community
# ONE-COMMAND SIMULATION SETUP - Run this once and it handles everything!

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=== ResQoUnity Simulation Setup ===${NC}"
echo -e "${YELLOW}This script will set up everything needed to run the simulation${NC}"

# Set default ROS_DISTRO if not set
if [ -z "$ROS_DISTRO" ]; then
    export ROS_DISTRO=humble
    echo -e "${YELLOW}Setting ROS_DISTRO to: $ROS_DISTRO${NC}"
fi

# Ensure we're in the right directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$SCRIPT_DIR"
echo -e "${BLUE}Working from: $(pwd)${NC}"

# Function to check if conda environment exists and is activated
check_conda_env() {
    if [[ "$CONDA_DEFAULT_ENV" == "orbit" ]]; then
        echo -e "${GREEN}✓ Conda orbit environment is active${NC}"
        return 0
    else
        echo -e "${RED}✗ Conda orbit environment not active${NC}"
        echo -e "${YELLOW}Attempting to activate orbit environment...${NC}"
        eval "$(conda shell.bash hook)"
        conda activate orbit
        return 0
    fi
}

# Function to find IsaacLab installation
find_isaaclab() {
    local paths=(
        "/home/kishi/IsaacLab_v0.3.1"
        "$HOME/IsaacLab_v0.3.1"
        "$HOME/IsaacLab"
        "$HOME/isaac-lab"
    )
    
    for path in "${paths[@]}"; do
        if [[ -d "$path" && (-f "$path/orbit.sh" || -f "$path/isaaclab.sh") ]]; then
            echo "$path"
            return 0
        fi
    done
    
    echo ""
    return 1
}

# Check prerequisites
echo -e "${BLUE}Checking prerequisites...${NC}"

# Check conda environment
check_conda_env

# Find IsaacLab
ISAACLAB_PATH=$(find_isaaclab)
if [[ -z "$ISAACLAB_PATH" ]]; then
    echo -e "${RED}✗ IsaacLab not found in common locations${NC}"
    echo -e "${YELLOW}Please ensure IsaacLab is installed${NC}"
    exit 1
fi
echo -e "${GREEN}✓ Found IsaacLab at: $ISAACLAB_PATH${NC}"

# Setup ROS environment
echo -e "${BLUE}Setting up ROS environment...${NC}"
source /opt/ros/${ROS_DISTRO}/setup.bash

# Build ROS workspaces
echo -e "${BLUE}Building ROS workspaces...${NC}"
if [[ -d "IsaacSim-ros_workspaces/${ROS_DISTRO}_ws" ]]; then
    cd IsaacSim-ros_workspaces/${ROS_DISTRO}_ws
    echo -e "${YELLOW}Building Isaac Sim ROS workspace...${NC}"
    rosdep install --from-paths src --ignore-src -r -y > /dev/null 2>&1 || true
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
    source install/setup.bash
    cd ../..
    echo -e "${GREEN}✓ Isaac Sim ROS workspace built${NC}"
else
    echo -e "${YELLOW}⚠ Isaac Sim ROS workspace not found, skipping${NC}"
fi

if [[ -d "go2_omniverse_ws" ]]; then
    cd go2_omniverse_ws
    echo -e "${YELLOW}Building Go2 workspace...${NC}"
    rosdep install --from-paths src --ignore-src -r -y > /dev/null 2>&1 || true
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
    source install/setup.bash
    cd ..
    echo -e "${GREEN}✓ Go2 workspace built${NC}"
else
    echo -e "${YELLOW}⚠ Go2 workspace not found, skipping${NC}"
fi

# Setup IsaacLab
echo -e "${BLUE}Setting up IsaacLab environment...${NC}"
cd "$ISAACLAB_PATH"

# Check if extensions are already installed by trying a simple import
echo -e "${YELLOW}Checking IsaacLab extensions...${NC}"
if ./orbit.sh -p -c "import omni.isaac.orbit; print('Extensions available')" 2>/dev/null; then
    echo -e "${GREEN}✓ IsaacLab extensions already installed and working${NC}"
else
    echo -e "${YELLOW}Installing IsaacLab extensions...${NC}"
    
    # Install dependencies silently
    echo -e "${YELLOW}Installing Python dependencies...${NC}"
    ./orbit.sh -p -m pip install toml setuptools wheel build 2>/dev/null || true
    
    # Try to install dependencies for extensions
    echo -e "${YELLOW}Installing extension dependencies...${NC}"
    ./orbit.sh --install-deps 2>/dev/null || true
    
    # Install extensions with error handling
    export SETUPTOOLS_USE_DISTUTILS=stdlib
    
    extensions=("omni.isaac.orbit" "omni.isaac.orbit_tasks" "omni.isaac.orbit_assets")
    for ext in "${extensions[@]}"; do
        echo -e "${YELLOW}Installing $ext...${NC}"
        if ./orbit.sh -p -m pip install -e "source/extensions/$ext" --config-settings editable_mode=compat --no-build-isolation 2>/dev/null; then
            echo -e "${GREEN}✓ $ext installed successfully${NC}"
        else
            echo -e "${YELLOW}⚠ $ext installation had issues, trying alternative method...${NC}"
            # Try without special flags
            ./orbit.sh -p -m pip install -e "source/extensions/$ext" 2>/dev/null || true
        fi
    done
fi

# Final verification
echo -e "${BLUE}Verifying setup...${NC}"
cd "$ISAACLAB_PATH"

# Test core Isaac Sim modules
if ./orbit.sh -p -c "import omni.isaac.core; print('Isaac Sim core: OK')" 2>/dev/null; then
    echo -e "${GREEN}✓ Isaac Sim core modules available${NC}"
else
    echo -e "${YELLOW}⚠ Isaac Sim core modules may need initialization${NC}"
fi

# Test IsaacLab modules
if ./orbit.sh -p -c "import omni.isaac.orbit; print('IsaacLab: OK')" 2>/dev/null; then
    echo -e "${GREEN}✓ IsaacLab modules available${NC}"
else
    echo -e "${YELLOW}⚠ IsaacLab modules may need initialization${NC}"
fi

# Source ROS workspaces for the simulation
source "$SCRIPT_DIR/IsaacSim-ros_workspaces/${ROS_DISTRO}_ws/install/setup.bash" 2>/dev/null || true
source "$SCRIPT_DIR/go2_omniverse_ws/install/setup.bash" 2>/dev/null || true

# Run the simulation
echo -e "${GREEN}=== Starting Simulation ===${NC}"
echo -e "${BLUE}Command: ./orbit.sh --sim $SCRIPT_DIR/main.py --robot_amount 1 --robot go2 --terrain flat${NC}"
echo -e "${YELLOW}Note: First run may take longer as Isaac Sim initializes extensions${NC}"

# Change back to IsaacLab directory for execution
cd "$ISAACLAB_PATH"
./orbit.sh --sim "$SCRIPT_DIR/main.py" --robot_amount 2 --robot go2 --terrain flat