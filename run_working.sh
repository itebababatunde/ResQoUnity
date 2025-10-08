#!/bin/bash

# Working script - proper Isaac Sim + IsaacLab + ROS setup

export ROS_DISTRO=humble

echo "[1/7] Deactivating conda (Isaac Sim requirement)..."
# Must deactivate conda FIRST before anything else
if [[ -n "$CONDA_DEFAULT_ENV" ]]; then
    echo "  Current conda env: $CONDA_DEFAULT_ENV - deactivating..."
    eval "$(conda shell.bash hook)"
    conda deactivate 2>/dev/null || true
    conda deactivate 2>/dev/null || true  # Sometimes needs 2x
fi

echo "[2/7] Patching Isaac Sim pydantic import..."
sed -i.bak 's/^    import pydantic$/    #import pydantic/' ~/.local/share/ov/pkg/isaac-sim-2023.1.1/kit/exts/omni.kit.helper.file_utils/omni/kit/helper/file_utils/extension.py 2>/dev/null || true

echo "[3/7] Setting up ROS environment..."
source /opt/ros/$ROS_DISTRO/setup.bash

echo "[4/7] Building ROS workspaces..."
cd ~/ResQoUnity/IsaacSim-ros_workspaces/${ROS_DISTRO}_ws
colcon build --cmake-args -Wno-dev 2>&1 | tail -5
source install/setup.bash

cd ~/ResQoUnity/go2_omniverse_ws  
colcon build --cmake-args -Wno-dev 2>&1 | tail -5
source install/setup.bash

echo "[5/7] Setting up Isaac Sim environment..."
export ISAACSIM_PATH="$HOME/.local/share/ov/pkg/isaac-sim-2023.1.1"
export ISAACSIM_PYTHON_EXE="${ISAACSIM_PATH}/python.sh"

echo "[6/7] Setting up IsaacLab Python paths and fixing dependencies..."
# Fix tensordict version incompatibility (Isaac Sim 2023.1.1 has old PyTorch)
echo "  Downgrading tensordict to match Isaac Sim's old PyTorch..."
${ISAACSIM_PATH}/python.sh -m pip install --quiet "tensordict<0.2.0"

# Install rsl_rl (RL library needed by orbit_tasks) - it's in the IsaacLab _isaac_sim directory
echo "  Installing rsl_rl from IsaacLab..."
if [ -d "$ISAACLAB_PATH/_isaac_sim/rsl_rl" ]; then
    ${ISAACSIM_PATH}/python.sh -m pip install --no-deps -e "$ISAACLAB_PATH/_isaac_sim/rsl_rl"
    echo "    rsl_rl installed from IsaacLab/_isaac_sim (no deps)"
elif [ -d "$HOME/rsl_rl" ]; then
    ${ISAACSIM_PATH}/python.sh -m pip install --no-deps -e "$HOME/rsl_rl"
    echo "    rsl_rl installed from ~/rsl_rl (no deps)"
else
    echo "    WARNING: rsl_rl not found, cloning from github..."
    cd /tmp
    rm -rf /tmp/rsl_rl
    git clone https://github.com/leggedrobotics/rsl_rl.git
    ${ISAACSIM_PATH}/python.sh -m pip install --no-deps -e /tmp/rsl_rl
    echo "    rsl_rl installed from github (no deps)"
fi

# Verify PyTorch wasn't broken - reinstall if needed
echo "  Verifying PyTorch integrity..."
if ! ${ISAACSIM_PATH}/python.sh -c "import torch; print('PyTorch OK')" 2>/dev/null | grep -q "PyTorch OK"; then
    echo "    WARNING: PyTorch broken, reinstalling torch==2.0.1..."
    ${ISAACSIM_PATH}/python.sh -m pip install --force-reinstall --no-deps torch==2.0.1
fi

# Add IsaacLab extensions directly to PYTHONPATH - no pip install needed!
export ISAACLAB_PATH="$HOME/IsaacLab_v0.3.1"
export PYTHONPATH="${ISAACLAB_PATH}/source/extensions/omni.isaac.orbit:${PYTHONPATH}"
export PYTHONPATH="${ISAACLAB_PATH}/source/extensions/omni.isaac.orbit_tasks:${PYTHONPATH}"
export PYTHONPATH="${ISAACLAB_PATH}/source/extensions/omni.isaac.orbit_assets:${PYTHONPATH}"
echo "  PYTHONPATH configured with IsaacLab extensions"

echo "[7/7] Running simulation with Isaac Sim Python..."
# Re-source ROS workspaces so Isaac Sim's Python can find ROS messages
source ~/ResQoUnity/IsaacSim-ros_workspaces/${ROS_DISTRO}_ws/install/setup.bash
source ~/ResQoUnity/go2_omniverse_ws/install/setup.bash

cd ~/ResQoUnity
echo "  Executing: ${ISAACSIM_PYTHON_EXE} main.py --robot_amount 1 --robot go2 --terrain flat"
${ISAACSIM_PYTHON_EXE} main.py --robot_amount 1 --robot go2 --terrain flat

