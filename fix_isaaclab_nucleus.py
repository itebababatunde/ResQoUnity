#!/usr/bin/env python3
"""
Patch IsaacLab to allow offline operation without Nucleus server.

This script modifies IsaacLab's assets.py to allow NUCLEUS_ASSET_ROOT_DIR to be None
instead of raising a RuntimeError.
"""

import os
import shutil
from datetime import datetime

# Find IsaacLab path
isaaclab_path = os.path.expanduser("~/IsaacLab_v0.3.1")
assets_file = f"{isaaclab_path}/source/extensions/omni.isaac.orbit/omni/isaac/orbit/utils/assets.py"

if not os.path.exists(assets_file):
    print(f"ERROR: Cannot find {assets_file}")
    exit(1)

# Backup original
backup_file = f"{assets_file}.backup_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
shutil.copy2(assets_file, backup_file)
print(f"✓ Backup created: {backup_file}")

# Read file
with open(assets_file, 'r') as f:
    lines = f.readlines()

# Modify lines 43-48 to use warning instead of error
modified = False
new_lines = []
i = 0
while i < len(lines):
    line = lines[i]
    
    # Look for the RuntimeError block
    if 'if NUCLEUS_ASSET_ROOT_DIR is None:' in line and not modified:
        print(f"✓ Found RuntimeError block at line {i+1}")
        
        # Keep the if statement
        new_lines.append(line)
        i += 1
        
        # Skip the error message and raise statement, replace with warning
        while i < len(lines) and 'raise RuntimeError' not in lines[i]:
            i += 1
        
        # Skip the raise line
        if i < len(lines) and 'raise RuntimeError' in lines[i]:
            i += 1
        
        # Add warning instead
        new_lines.append('    carb.log_warn("Nucleus server not available - using offline mode")\n')
        new_lines.append('    # Patched by ResQoUnity: Allow offline operation\n')
        modified = True
    else:
        new_lines.append(line)
        i += 1

if not modified:
    print("WARNING: Could not find the RuntimeError block to patch!")
    print("File may have already been patched or structure changed.")
    exit(1)

# Write modified file
with open(assets_file, 'w') as f:
    f.writelines(new_lines)

print(f"✓ Patched {assets_file}")
print("✓ IsaacLab now allows offline operation without Nucleus")
print(f"\nTo restore original: cp {backup_file} {assets_file}")

