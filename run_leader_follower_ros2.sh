#!/bin/bash

# Leader-Follower ROS2 Node Launcher
#
# Runs the standalone leader-follower coordination node.
# The simulation must already be running (./run_sim.sh) in another terminal.
#
# Usage:
#   Terminal 1:  ./run_sim.sh                    # starts Isaac Sim + Go2 + drone
#   Terminal 2:  ./run_leader_follower_ros2.sh   # starts this coordination node

set -e

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}   Leader-Follower ROS2 Coordination Node${NC}"
echo -e "${BLUE}   Drone (Leader) + 2x Go2 Dogs (Followers)${NC}"
echo -e "${BLUE}   RViz2 visualization enabled${NC}"
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

# Match the simulation's DDS settings so nodes can discover each other
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
echo -e "${GREEN}ROS2 DDS: FastRTPS, DOMAIN_ID=0, LOCALHOST_ONLY=1${NC}"

# Source project workspaces (for custom message types)
if [[ -f "$SCRIPT_DIR/IsaacSim-ros_workspaces/${ROS_DISTRO}_ws/install/setup.bash" ]]; then
    source "$SCRIPT_DIR/IsaacSim-ros_workspaces/${ROS_DISTRO}_ws/install/setup.bash"
fi
if [[ -f "$SCRIPT_DIR/go2_omniverse_ws/install/setup.bash" ]]; then
    source "$SCRIPT_DIR/go2_omniverse_ws/install/setup.bash"
fi

echo -e "${YELLOW}Ensure the simulation is running in another terminal (./run_sim.sh)${NC}"
echo -e "${GREEN}Starting leader-follower node...${NC}"
echo ""

# Launch RViz2 in background with saved config
RVIZ_PID=""
if command -v rviz2 &>/dev/null; then
    if [[ -f "$SCRIPT_DIR/leader_follower.rviz" ]]; then
        echo -e "${GREEN}Launching RViz2...${NC}"
        rviz2 -d "$SCRIPT_DIR/leader_follower.rviz" &
        RVIZ_PID=$!
    else
        echo -e "${YELLOW}RViz2 config not found, skipping visualization${NC}"
    fi
else
    echo -e "${YELLOW}rviz2 not found, skipping visualization${NC}"
fi

# Cleanup trap to kill RViz2 on exit
cleanup() {
    [[ -n "$RVIZ_PID" ]] && kill "$RVIZ_PID" 2>/dev/null
}
trap cleanup EXIT

python3 "$SCRIPT_DIR/leader_follower_ros2_node.py" "$@"
