#!/bin/bash
# Comprehensive indentation fix for omniverse_sim.py

cd /Users/iteoluwakishi/ResQoUnity

echo "Fixing all indentation issues..."

# Restore clean version from git
git checkout omniverse_sim.py

# Fix section 1 (lines 561-567, 577)
sed -i '' -e '561,564s/^        /            /' \
          -e '567s/^        /            /' \
          -e '577s/^        /            /' \
          omniverse_sim.py

# Verify
python3 -m py_compile omniverse_sim.py && echo "✅ ALL FIXED!" || echo "❌ Still has errors"

