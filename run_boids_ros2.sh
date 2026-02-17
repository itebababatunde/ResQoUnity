#!/bin/bash

# Boids Swarm ROS2 Node Launcher
#
# Runs the standalone boids swarm coordination node.
# The simulation must already be running (./run_sim.sh) in another terminal.
#
# Usage:
#   Terminal 1:  ./run_sim.sh              # starts Isaac Sim + 2x Go2 + drone
#   Terminal 2:  ./run_boids_ros2.sh       # starts this coordination node

set -e

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}   Boids Swarm ROS2 Coordination Node${NC}"
echo -e "${BLUE}   Drone (Leader) + 2x Go2 Dogs (Boids)${NC}"
echo -e "${BLUE}============================================${NC}"

# Source ROS2
export ROS_DISTRO=${ROS_DISTRO:-humble}
if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    echo -e "${GREEN}Sourced /opt/ros/${ROS_DISTRO}/setup.bash${NC}"
else
    echo -e "${RED}ROS2 ${ROS_DISTRO} not found at /opt/ros/${ROS_DISTRO}${NC}"
    exit 1
fi

# Source project workspaces (for custom message types)
if [[ -f "$SCRIPT_DIR/IsaacSim-ros_workspaces/${ROS_DISTRO}_ws/install/setup.bash" ]]; then
    source "$SCRIPT_DIR/IsaacSim-ros_workspaces/${ROS_DISTRO}_ws/install/setup.bash"
fi
if [[ -f "$SCRIPT_DIR/go2_omniverse_ws/install/setup.bash" ]]; then
    source "$SCRIPT_DIR/go2_omniverse_ws/install/setup.bash"
fi

echo -e "${YELLOW}Ensure the simulation is running in another terminal (./run_sim.sh)${NC}"
echo -e "${GREEN}Starting boids swarm node...${NC}"
echo ""

python3 "$SCRIPT_DIR/boids_ros2_node.py" "$@"
