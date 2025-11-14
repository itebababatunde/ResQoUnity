# Drone Diagnostics Guide

## What to Do:

### 1. Start the simulation:
```bash
cd ~/ResQoUnity
./start_simulation.sh go2 1 flat office
```

### 2. Watch the console output for these diagnostic messages:

#### A. At Startup (once):
```
[DIAG] DRONE INITIALIZATION COMPLETE
```
- Confirms drone was found and initialized
- Shows initial position, mass, mode

#### B. During Simulation (every 1 second):
```
[DIAG FRAME 60] CONTROL LOOP RUNNING
[DIAG FRAME 120] CONTROL LOOP RUNNING
...
```
- Confirms control code is executing
- Shows current position, target, error, velocity

#### C. Control Application Details (every 1 second):
```
Calculated Control Values:
  Desired Vel:  [...]
  Forces:       [...]
  New Velocity: [...]

After set_velocities():
  ✅ Position CHANGED by: [...]
  OR
  ❌ Position UNCHANGED
```

---

## 3. Send a Takeoff Command:

In a **new terminal**:
```bash
ros2 service call /drone/takeoff std_srvs/srv/Trigger
```

---

## 4. Check ROS2 Odometry:

In the **same new terminal**:
```bash
ros2 topic echo /drone/odom --once | grep -A3 "position:"
```

Wait 5 seconds, then run again to see if position changed.

---

## 5. Report Results:

**Paste here:**

1. The `[DIAG] DRONE INITIALIZATION COMPLETE` block
2. At least 3 `[DIAG FRAME XXX]` blocks (showing 3 seconds of control)
3. Whether you see `✅ Position CHANGED` or `❌ Position UNCHANGED`
4. The odometry position before and after takeoff

---

## What Each Result Means:

### If NO `[DIAG]` messages appear:
**Problem:** Control loop not running  
**Fix:** Controller not initialized or exception in control block

### If `[DIAG]` messages appear but always show `❌ Position UNCHANGED`:
**Problem:** `set_velocities()` is being ignored  
**Fix:** Need to use direct position integration or different approach

### If `✅ Position CHANGED` in diagnostics but ROS2 shows frozen:
**Problem:** Odometry publishing stale cache  
**Fix:** Update cache timing

### If position changes in BOTH diagnostics and ROS2:
**Problem:** PID tuning or gains too low  
**Fix:** Increase controller gains

