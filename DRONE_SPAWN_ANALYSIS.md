# Drone Spawn Issue - Root Cause Analysis

## Problem
Drone spawns at **Z=0.075m** (on ground) instead of **Z=2.5m** (hovering)

## Root Cause Found

### From Log Analysis (`drone_sim_20251116_200631.log`)

```
Line 126: [CALIBRATION] Data collection initialized
Line 127: [CALIBRATION] Phase 1: Free-fall test (3 seconds)
Line 129: [DBG ODOM] Published /robot0/odom: pos=(0.003, 0.009, 0.075)
Line 130: [CALIBRATION] Test 1 complete. Collected 63 samples
```

**YOU RAN WITH `--calibrate` FLAG!**

##What Happened

1. **Calibration Mode Activated**: The `--calibrate` flag was passed to the simulation
2. **Test 1 - Free Fall**: Calibration's first test DISABLES all control for 3 seconds
3. **Drone Fell**: The drone fell from 2.5m spawn height to the ground (0.075m)
4. **Test 2 Started**: Hover force sweep began with drone already on ground
5. **Test Script Ran**: You then ran `test_drone_diagnostics.py` which found the drone at ground level

### Why It Switched to LANDING Mode

```
Line 159: [Drone 0] Mode: IDLE → LANDING
```

During calibration Test 2 (hover force sweep), something triggered the land service. This is likely:
- **The diagnostic test script calling the land service** while calibration was still running
- OR a stale ROS2 message from a previous session

## The Solution

### DON'T run with `--calibrate` for normal flight testing!

**CORRECT command for normal flight:**
```bash
./start_simulation.sh drone 1 flat office
```

**WRONG command (runs calibration tests):**
```bash
./start_simulation.sh drone 1 flat office --calibrate  # ❌ This ran calibration!
```

### Calibration is ONLY for measuring physics parameters

Calibration mode:
- Runs automated 113-second test sequence
- **Intentionally** lets drone fall (Test 1)
- **Intentionally** applies different forces (Test 2)  
- **Intentionally** tests different PID gains (Test 3)
- Is NOT for normal flight!

## What To Do Now

### 1. Run WITHOUT Calibration

```bash
# Kill any running simulation
pkill -f isaac

# Start fresh simulation WITHOUT --calibrate
./start_simulation.sh drone 1 flat office
```

### 2. Then Run Diagnostic Tests

```bash
# In another terminal
python3 test_drone_diagnostics.py
```

### Expected Behavior (WITHOUT --calibrate)

1. ✅ Drone spawns at Z=2.5m (hovering)
2. ✅ Drone stays in IDLE mode initially
3. ✅ After 2 frames, switches to LOITER (holds position)
4. ✅ Diagnostic tests can then command takeoff/landing/movement

## When To Use --calibrate

Only use calibration when you want to:
- Measure actual drone mass from USD
- Find optimal hover force  
- Auto-tune PID gains

```bash
# Run calibration (takes ~2 minutes)
./start_simulation.sh drone 1 flat office --calibrate

# Wait for "All tests complete!"
# Then analyze results
python3 calibrate_drone_physics.py --analyze calibration_data.json

# Review and apply recommended parameters
cat CALIBRATION_RESULTS.md
```

## Secondary Issue: IDLE Mode Hover

There WAS a bug in IDLE mode (fixed in commit 3693512):
- Before: `desired_vz = 0.0` → drone would fall
- After: `desired_vz = -current_vel[2]` → actively cancels downward velocity

This fix ensures the drone hovers when in IDLE, but **only matters if the drone starts in the air!**

## Summary

- ❌ **Problem**: You ran `--calibrate` which intentionally dropped the drone
- ✅ **Solution**: Run WITHOUT `--calibrate` for normal flight
- ✅ **IDLE mode fix**: Already applied (commit 3693512)
- ✅ **Next step**: Run simulation normally and retest

The drone WILL spawn at 2.5m when you run without calibration mode!

