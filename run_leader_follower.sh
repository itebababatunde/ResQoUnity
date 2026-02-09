#!/bin/bash

# Leader-Follower Simulation Launcher
# Runs the drone-dog leader-follower system through IsaacLab

set -e

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
ISAACLAB_PATH="/home/kishi/IsaacLab_v0.3.1"

echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}   Leader-Follower Simulation${NC}"
echo -e "${BLUE}   Drone (Leader) + Go2 Dog (Follower)${NC}"
echo -e "${BLUE}============================================${NC}"

# Check IsaacLab exists
if [[ ! -d "$ISAACLAB_PATH" ]]; then
    echo -e "${RED}Error: IsaacLab not found at $ISAACLAB_PATH${NC}"
    exit 1
fi

# Activate conda environment
if [[ "$CONDA_DEFAULT_ENV" != "orbit" ]]; then
    echo -e "${YELLOW}Activating orbit conda environment...${NC}"
    eval "$(conda shell.bash hook)"
    conda activate orbit || { echo -e "${RED}Failed to activate conda env${NC}"; exit 1; }
fi

echo -e "${GREEN}Conda environment: $CONDA_DEFAULT_ENV${NC}"

# Setup ROS (optional, for logging)
export ROS_DISTRO=${ROS_DISTRO:-humble}
source /opt/ros/${ROS_DISTRO}/setup.bash 2>/dev/null || true
source "$SCRIPT_DIR/go2_omniverse_ws/install/setup.bash" 2>/dev/null || true

# Parse command line args
EXTRA_ARGS="$@"
if [[ -z "$EXTRA_ARGS" ]]; then
    EXTRA_ARGS="--terrain flat"
fi

echo -e "${YELLOW}Running simulation with args: $EXTRA_ARGS${NC}"

# Run simulation through IsaacLab
cd "$ISAACLAB_PATH"
./orbit.sh --sim "$SCRIPT_DIR/leader_follower_sim.py" $EXTRA_ARGS
