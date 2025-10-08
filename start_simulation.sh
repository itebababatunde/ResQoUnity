#!/bin/bash
################################################################################
# Go2 Robot Simulation Startup Script
# 
# This script handles all environment setup and launches the Isaac Sim
# simulation with the Go2 robot in one command.
#
# Usage: ./start_simulation.sh [--robot_amount N] [--robot ROBOT] [--terrain TYPE]
################################################################################

set -e  # Exit on error

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Go2 Robot Simulation Startup${NC}"
echo -e "${GREEN}========================================${NC}"

# All configuration (robot amount, type, terrain) should be set in omniverse_sim.py
# This script just handles environment setup and launches the simulation

echo -e "${GREEN}[1/8]${NC} Deactivating conda environment..."
# Must deactivate conda first - Isaac Sim doesn't work with conda active
if [[ -n "$CONDA_DEFAULT_ENV" ]]; then
    echo "  Current conda env: $CONDA_DEFAULT_ENV - deactivating..."
    eval "$(conda shell.bash hook)"
    conda deactivate 2>/dev/null || true
    conda deactivate 2>/dev/null || true  # Sometimes needs 2x
    echo -e "  ${GREEN}✓${NC} Conda deactivated"
else
    echo -e "  ${GREEN}✓${NC} No conda environment active"
fi

echo -e "${GREEN}[2/8]${NC} Patching Isaac Sim pydantic import..."
# Fix pydantic import issue in Isaac Sim
sed -i.bak 's/^    import pydantic$/    #import pydantic/' \
    ~/.local/share/ov/pkg/isaac-sim-2023.1.1/kit/exts/omni.kit.helper.file_utils/omni/kit/helper/file_utils/extension.py \
    2>/dev/null || true
echo -e "  ${GREEN}✓${NC} Pydantic patched"

echo -e "${GREEN}[3/8]${NC} Setting up ROS environment..."
export ROS_DISTRO=humble
source /opt/ros/$ROS_DISTRO/setup.bash
echo -e "  ${GREEN}✓${NC} ROS $ROS_DISTRO sourced"

echo -e "${GREEN}[4/8]${NC} Building ROS workspaces..."
# Build Isaac Sim ROS workspace
cd ~/ResQoUnity/IsaacSim-ros_workspaces/${ROS_DISTRO}_ws
if colcon build --cmake-args -Wno-dev 2>&1 | tail -5 | grep -q "finished"; then
    echo -e "  ${GREEN}✓${NC} Isaac Sim ROS workspace built"
else
    echo -e "  ${YELLOW}⚠${NC} Isaac Sim ROS workspace may have warnings"
fi
source install/setup.bash

# Build Go2 ROS workspace
cd ~/ResQoUnity/go2_omniverse_ws
if colcon build --cmake-args -Wno-dev 2>&1 | tail -5 | grep -q "finished"; then
    echo -e "  ${GREEN}✓${NC} Go2 ROS workspace built"
else
    echo -e "  ${YELLOW}⚠${NC} Go2 ROS workspace may have warnings"
fi
source install/setup.bash

echo -e "${GREEN}[5/8]${NC} Setting up Isaac Sim environment..."
# Try to find Isaac Sim installation automatically
if [ -d "$HOME/.local/share/ov/pkg/isaac-sim-2023.1.1" ]; then
    export ISAACSIM_PATH="$HOME/.local/share/ov/pkg/isaac-sim-2023.1.1"
elif [ -d "$HOME/isaac-sim-2023.1.1" ]; then
    export ISAACSIM_PATH="$HOME/isaac-sim-2023.1.1"
else
    # Try to find any isaac-sim installation
    FOUND_PATH=$(find $HOME/.local/share/ov/pkg -maxdepth 1 -name "isaac-sim-*" 2>/dev/null | head -1)
    if [ -n "$FOUND_PATH" ]; then
        export ISAACSIM_PATH="$FOUND_PATH"
        echo -e "  ${YELLOW}⚠${NC} Using Isaac Sim at: $ISAACSIM_PATH"
    else
        echo -e "  ${RED}✗${NC} Isaac Sim not found!"
        echo -e "  ${YELLOW}Please set ISAACSIM_PATH manually${NC}"
        exit 1
    fi
fi
export ISAACSIM_PYTHON_EXE="${ISAACSIM_PATH}/python.sh"
echo -e "  ${GREEN}✓${NC} Isaac Sim found at: $ISAACSIM_PATH"

echo -e "${GREEN}[6/8]${NC} Setting up IsaacLab and dependencies..."

# Fix tensordict version incompatibility
if ! ${ISAACSIM_PATH}/python.sh -c "from tensordict import TensorDict" 2>/dev/null; then
    echo "  Installing compatible tensordict..."
    ${ISAACSIM_PATH}/python.sh -m pip install --quiet "tensordict<0.2.0"
    echo -e "  ${GREEN}✓${NC} tensordict installed"
else
    echo -e "  ${GREEN}✓${NC} tensordict already compatible"
fi

# Install rsl_rl (RL library)
# Try to find IsaacLab installation automatically
if [ -d "$HOME/IsaacLab_v0.3.1" ]; then
    export ISAACLAB_PATH="$HOME/IsaacLab_v0.3.1"
elif [ -d "$HOME/IsaacLab" ]; then
    export ISAACLAB_PATH="$HOME/IsaacLab"
else
    FOUND_LAB=$(find $HOME -maxdepth 2 -type d -name "IsaacLab*" 2>/dev/null | head -1)
    if [ -n "$FOUND_LAB" ]; then
        export ISAACLAB_PATH="$FOUND_LAB"
        echo -e "  ${YELLOW}⚠${NC} Using IsaacLab at: $ISAACLAB_PATH"
    else
        echo -e "  ${RED}✗${NC} IsaacLab not found!"
        echo -e "  ${YELLOW}Please set ISAACLAB_PATH manually${NC}"
        exit 1
    fi
fi
if ! ${ISAACSIM_PATH}/python.sh -c "import rsl_rl" 2>/dev/null; then
    echo "  Installing rsl_rl..."
    if [ -d "$ISAACLAB_PATH/_isaac_sim/rsl_rl" ]; then
        ${ISAACSIM_PATH}/python.sh -m pip install --quiet --no-deps -e "$ISAACLAB_PATH/_isaac_sim/rsl_rl"
        echo -e "  ${GREEN}✓${NC} rsl_rl installed from IsaacLab"
    elif [ -d "$HOME/rsl_rl" ]; then
        ${ISAACSIM_PATH}/python.sh -m pip install --quiet --no-deps -e "$HOME/rsl_rl"
        echo -e "  ${GREEN}✓${NC} rsl_rl installed from ~/rsl_rl"
    else
        echo "  Cloning rsl_rl from GitHub..."
        cd /tmp
        rm -rf /tmp/rsl_rl
        git clone --quiet https://github.com/leggedrobotics/rsl_rl.git
        ${ISAACSIM_PATH}/python.sh -m pip install --quiet --no-deps -e /tmp/rsl_rl
        echo -e "  ${GREEN}✓${NC} rsl_rl installed from GitHub"
    fi
else
    echo -e "  ${GREEN}✓${NC} rsl_rl already installed"
fi

# Verify PyTorch integrity
echo "  Verifying PyTorch..."
if ! ${ISAACSIM_PATH}/python.sh -c "import torch; torch.cuda.is_available()" 2>/dev/null | grep -q "True"; then
    echo -e "  ${YELLOW}⚠${NC} PyTorch may need attention, but continuing..."
fi

echo -e "${GREEN}[7/8]${NC} Configuring IsaacLab Python paths..."
# Add IsaacLab extensions to PYTHONPATH (no pip install needed)
export PYTHONPATH="${ISAACLAB_PATH}/source/extensions/omni.isaac.orbit:${PYTHONPATH}"
export PYTHONPATH="${ISAACLAB_PATH}/source/extensions/omni.isaac.orbit_tasks:${PYTHONPATH}"
export PYTHONPATH="${ISAACLAB_PATH}/source/extensions/omni.isaac.orbit_assets:${PYTHONPATH}"
echo -e "  ${GREEN}✓${NC} PYTHONPATH configured with IsaacLab extensions"

echo -e "${GREEN}[8/8]${NC} Launching simulation..."
# Re-source ROS workspaces for Isaac Sim Python
source ~/ResQoUnity/IsaacSim-ros_workspaces/${ROS_DISTRO}_ws/install/setup.bash
source ~/ResQoUnity/go2_omniverse_ws/install/setup.bash

cd ~/ResQoUnity
echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Starting Isaac Sim${NC}"
echo -e "  Configuration: ${YELLOW}omniverse_sim.py${NC}"
echo -e "  Note: ${YELLOW}Enable extensions via Window→Extensions if needed${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# Note: If you need Pegasus or other extensions, enable them manually via GUI first
# The replicator.agent.core extension needs to be enabled before Pegasus can be installed
${ISAACSIM_PYTHON_EXE} main.py

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Simulation ended${NC}"
echo -e "${GREEN}========================================${NC}"

