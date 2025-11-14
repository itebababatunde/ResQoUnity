#!/bin/bash
# Patch IsaacLab to disable Nucleus requirement for offline operation

set -e

ISAACLAB_PATH="${ISAACLAB_PATH:-$HOME/IsaacLab_v0.3.1}"
ASSETS_FILE="${ISAACLAB_PATH}/source/extensions/omni.isaac.orbit/omni/isaac/orbit/utils/assets.py"

echo "Patching IsaacLab assets.py to disable Nucleus check..."

if [ ! -f "$ASSETS_FILE" ]; then
    echo "ERROR: Cannot find $ASSETS_FILE"
    exit 1
fi

# Backup original file
cp "$ASSETS_FILE" "${ASSETS_FILE}.backup_$(date +%Y%m%d_%H%M%S)"

# Comment out the problematic import line that calls get_assets_root_path()
# This typically happens around line 27-46
sed -i 's/^\(ISAAC_NUCLEUS_DIR.*get_assets_root_path.*\)$/# \1  # Patched: Nucleus not required/' "$ASSETS_FILE"
sed -i 's/^\(NVIDIA_NUCLEUS_DIR.*get_assets_root_path.*\)$/# \1  # Patched: Nucleus not required/' "$ASSETS_FILE"
sed -i 's/^\(.*= get_assets_root_path.*\)$/# \1  # Patched: Nucleus not required/' "$ASSETS_FILE"

# Set these to None or empty strings instead
cat >> "$ASSETS_FILE" << 'EOF'

# Patched by ResQoUnity - Nucleus not required for local assets
if 'ISAAC_NUCLEUS_DIR' not in dir():
    ISAAC_NUCLEUS_DIR = None
if 'NVIDIA_NUCLEUS_DIR' not in dir():
    NVIDIA_NUCLEUS_DIR = None
EOF

echo "✓ Patch applied successfully!"
echo "  Backup saved to: ${ASSETS_FILE}.backup_*"
echo ""
echo "If you need to restore:"
echo "  cp ${ASSETS_FILE}.backup_* $ASSETS_FILE"

