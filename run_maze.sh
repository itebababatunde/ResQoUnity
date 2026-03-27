#!/bin/bash
# run_maze.sh — Launch the aerial-assisted maze navigation scenario
#
# Usage:
#   Terminal 1:  ./run_maze.sh sim   # Isaac Sim + Go2 + world drone + maze walls
#   Terminal 2:  ./run_maze.sh node  # Maze coordination ROS2 node

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
MODE="${1:-}"
shift 2>/dev/null || true  # consume mode arg so "$@" only has extra args

if [[ -z "$MODE" ]]; then
    echo -e "${YELLOW}Usage: $0 {sim|node}${NC}"
    echo "  Terminal 1:  ./run_maze.sh sim"
    echo "  Terminal 2:  ./run_maze.sh node"
    exit 1
fi

# ---------------------------------------------------------------------------
# SIM mode — identical setup to start_simulation.sh, maze-specific launch
# ---------------------------------------------------------------------------
if [[ "$MODE" == "sim" ]]; then
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}Maze Navigation — Isaac Sim${NC}"
    echo -e "${GREEN}  1x Go2 Dog + World Drone + Maze Walls${NC}"
    echo -e "${GREEN}========================================${NC}"

    # [1] Deactivate conda (Isaac Sim incompatible with conda)
    if [[ -n "$CONDA_DEFAULT_ENV" ]]; then
        echo "  Deactivating conda env: $CONDA_DEFAULT_ENV"
        eval "$(conda shell.bash hook)"
        conda deactivate 2>/dev/null || true
        conda deactivate 2>/dev/null || true
    fi

    # [2] Patch pydantic import
    sed -i.bak 's/^    import pydantic$/    #import pydantic/' \
        ~/.local/share/ov/pkg/isaac-sim-2023.1.1/kit/exts/omni.kit.helper.file_utils/omni/kit/helper/file_utils/extension.py \
        2>/dev/null || true

    # [3] ROS environment
    export ROS_DISTRO=humble
    source /opt/ros/$ROS_DISTRO/setup.bash
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export ROS_DOMAIN_ID=0
    export ROS_LOCALHOST_ONLY=1
    echo -e "  ${GREEN}✓${NC} ROS $ROS_DISTRO sourced"

    # [4] Build and source ROS workspaces
    cd "$SCRIPT_DIR/IsaacSim-ros_workspaces/${ROS_DISTRO}_ws"
    colcon build --cmake-args -Wno-dev 2>&1 | tail -3 || true
    source install/setup.bash
    cd "$SCRIPT_DIR/go2_omniverse_ws"
    colcon build --cmake-args -Wno-dev 2>&1 | tail -3 || true
    source install/setup.bash
    cd "$SCRIPT_DIR"

    # [5] Isaac Sim path + asset root (same vars as start_simulation.sh)
    if [[ -d "$HOME/.local/share/ov/pkg/isaac-sim-2023.1.1" ]]; then
        export ISAACSIM_PATH="$HOME/.local/share/ov/pkg/isaac-sim-2023.1.1"
    else
        FOUND=$(find "$HOME/.local/share/ov/pkg" -maxdepth 1 -name "isaac-sim-*" 2>/dev/null | head -1)
        [[ -n "$FOUND" ]] || { echo -e "${RED}Isaac Sim not found!${NC}"; exit 1; }
        export ISAACSIM_PATH="$FOUND"
    fi
    export ISAACSIM_PYTHON_EXE="${ISAACSIM_PATH}/python.sh"
    export ISAACSIM_ASSET_ROOT="${ISAACSIM_PATH}/assets"
    export ISAAC_ASSET_ROOT_PATH="${ISAACSIM_PATH}/assets"
    export OMNI_KIT_ACCEPT_EULA=YES
    echo -e "  ${GREEN}✓${NC} Isaac Sim: $ISAACSIM_PATH"
    echo -e "  ${GREEN}✓${NC} Asset root: $ISAACSIM_ASSET_ROOT"

    # [6] IsaacLab path
    if [[ -d "$HOME/IsaacLab_v0.3.1" ]]; then
        export ISAACLAB_PATH="$HOME/IsaacLab_v0.3.1"
    elif [[ -d "$HOME/IsaacLab" ]]; then
        export ISAACLAB_PATH="$HOME/IsaacLab"
    else
        FOUND=$(find "$HOME" -maxdepth 2 -type d -name "IsaacLab*" 2>/dev/null | head -1)
        [[ -n "$FOUND" ]] || { echo -e "${RED}IsaacLab not found!${NC}"; exit 1; }
        export ISAACLAB_PATH="$FOUND"
    fi
    echo -e "  ${GREEN}✓${NC} IsaacLab: $ISAACLAB_PATH"

    # [7] Fix tensordict version
    if ! ${ISAACSIM_PYTHON_EXE} -c "from tensordict import TensorDict" 2>/dev/null; then
        ${ISAACSIM_PYTHON_EXE} -m pip install --quiet "tensordict<0.2.0"
    fi

    # [8] Install rsl_rl v2.3.2 (pinned — v3+ has incompatible API)
    if ! ${ISAACSIM_PYTHON_EXE} -c "from rsl_rl.runners import OnPolicyRunner" 2>/dev/null; then
        echo "  Installing rsl_rl v2.3.2..."
        if [[ -d "$ISAACLAB_PATH/_isaac_sim/rsl_rl" ]]; then
            ${ISAACSIM_PYTHON_EXE} -m pip install --quiet --no-deps -e "$ISAACLAB_PATH/_isaac_sim/rsl_rl"
        elif [[ -d "$HOME/rsl_rl" ]]; then
            ${ISAACSIM_PYTHON_EXE} -m pip install --quiet --no-deps -e "$HOME/rsl_rl"
        else
            cd /tmp && rm -rf rsl_rl
            git clone --quiet https://github.com/leggedrobotics/rsl_rl.git
            git -C /tmp/rsl_rl checkout 270cb89  # v2.3.2
            ${ISAACSIM_PYTHON_EXE} -m pip install --quiet --no-deps -e /tmp/rsl_rl
            cd "$SCRIPT_DIR"
        fi
    fi

    # [9] Patch IsaacLab for offline operation (run once)
    if ! grep -q "Patched by ResQoUnity" \
        "${ISAACLAB_PATH}/source/extensions/omni.isaac.orbit/omni/isaac/orbit/utils/assets.py" 2>/dev/null; then
        ${ISAACSIM_PYTHON_EXE} ~/ResQoUnity/fix_isaaclab_nucleus.py
    fi

    # [10] IsaacLab extensions on PYTHONPATH
    export PYTHONPATH="${ISAACLAB_PATH}/source/extensions/omni.isaac.orbit:${PYTHONPATH}"
    export PYTHONPATH="${ISAACLAB_PATH}/source/extensions/omni.isaac.orbit_tasks:${PYTHONPATH}"
    export PYTHONPATH="${ISAACLAB_PATH}/source/extensions/omni.isaac.orbit_assets:${PYTHONPATH}"
    echo -e "  ${GREEN}✓${NC} PYTHONPATH configured"

    # [11] Re-source ROS workspaces
    source ~/ResQoUnity/IsaacSim-ros_workspaces/${ROS_DISTRO}_ws/install/setup.bash
    source ~/ResQoUnity/go2_omniverse_ws/install/setup.bash
    export ROS_DOMAIN_ID=0
    export ROS_LOCALHOST_ONLY=1

    mkdir -p "$SCRIPT_DIR/logs/simulation"
    LOG_FILE="$SCRIPT_DIR/logs/simulation/maze_$(date +%Y%m%d_%H%M%S).log"

    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}Launching maze sim${NC}"
    echo -e "  Robot:       ${YELLOW}1x Go2 + World Drone${NC}"
    echo -e "  Environment: ${YELLOW}maze${NC}"
    echo -e "  Log:         ${YELLOW}$LOG_FILE${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""

    cd "$SCRIPT_DIR"

    # (Re)generate maze.usda before launching so USD always matches the seed
    MAZE_SEED="${MAZE_SEED:-42}"
    echo -e "  ${GREEN}Generating maze.usda (seed=$MAZE_SEED)...${NC}"
    ${ISAACSIM_PYTHON_EXE} generate_maze_usd.py "$MAZE_SEED" 2>&1 | grep '\[MazeUSD\]'

    ${ISAACSIM_PYTHON_EXE} -u main.py \
        --robot go2 \
        --robot_amount 1 \
        --terrain flat \
        --custom_env maze \
        --seed "$MAZE_SEED" \
        --cpu \
        "$@" 2>&1 | tee -a "$LOG_FILE"

# ---------------------------------------------------------------------------
# NODE mode
# ---------------------------------------------------------------------------
elif [[ "$MODE" == "node" ]]; then
    echo -e "${BLUE}============================================${NC}"
    echo -e "${BLUE}   Maze Navigation — ROS2 Coordination     ${NC}"
    echo -e "${BLUE}============================================${NC}"

    export ROS_DISTRO=humble
    source /opt/ros/$ROS_DISTRO/setup.bash
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export ROS_DOMAIN_ID=0
    export ROS_LOCALHOST_ONLY=1

    source "$SCRIPT_DIR/IsaacSim-ros_workspaces/${ROS_DISTRO}_ws/install/setup.bash" 2>/dev/null || true
    source "$SCRIPT_DIR/go2_omniverse_ws/install/setup.bash" 2>/dev/null || true

    echo -e "${YELLOW}Ensure the simulation is running: ./run_maze.sh sim${NC}"
    echo -e "${GREEN}Starting maze coordination node...${NC}"
    echo ""

    cd "$SCRIPT_DIR"
    python3 maze_ros2_node.py "$@"

else
    echo -e "${RED}Unknown mode: $MODE${NC}"
    echo "Usage: $0 {sim|node}"
    exit 1
fi
