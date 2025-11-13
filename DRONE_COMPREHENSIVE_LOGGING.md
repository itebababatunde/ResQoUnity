# Drone Comprehensive Logging - Debug Guide

## What Was Added

### 1. **DroneDebugLogger Class** (`drone_debug_logger.py`)
A dedicated logging system that tracks:
- ✅ Initialization events
- ✅ Arm/disarm events  
- ✅ Mode changes
- ✅ Frame-by-frame state (position, velocity, errors, PID outputs)
- ✅ Velocity application verification
- ✅ Warnings for stuck drone, altitude errors
- ✅ Session summaries

### 2. **Integration in omniverse_sim.py**
Logging added at critical points:
- Controller initialization
- Arm/disarm callbacks
- Main control loop (every frame)
- Velocity application and verification

---

## How to Read the Logs

### **At Startup:**
```
================================================================================
DRONE DEBUG LOGGER INITIALIZED FOR world_drone
================================================================================

[INIT] Drone spawned at position: (0.000, 0.000, 2.500)
[INIT] Initial velocity: (0.000, 0.000, 0.000)
[INIT] Initial mode: DISARMED
[INIT] Ready for control
```

### **When Armed:**
```
********************************************************************************
[ARM EVENT] Drone ARMED
********************************************************************************
```

### **When Takeoff Called:**
```
[TAKEOFF] Current altitude: 2.50m → Target: 4.00m
[TAKEOFF] Need to climb: 1.50m
```

### **Every Second (Frame Logging):**
```
────────────────────────────────────────────────────────────────────────────────
[FRAME 60] Δt = 1.00s
────────────────────────────────────────────────────────────────────────────────
  State      : Armed=True, Mode=POSITION
  Position   : (  0.000,   0.000,   2.500)
  Velocity   : (  0.000,   0.000,   0.000)  ← CURRENT velocity from sensors
  Target Pos : (  0.000,   0.000,   4.000)
  Error      : (  0.000,   0.000,   1.500) = 1.500m
  PID Output : (  0.000,   0.000,   1.200) m/s  ← What PID calculated
  Applied Vel: (  0.000,   0.000,   1.200) m/s  ← What was sent to physics
  ✅ Velocity command APPLIED correctly
────────────────────────────────────────────────────────────────────────────────
```

---

## Diagnostic Indicators

### ✅ **Everything Working:**
```
  ✅ Velocity command APPLIED correctly
```
- PID Output matches Applied Vel
- Drone is moving (velocity non-zero)
- Position approaching target

### ⚠️ **Drone Stuck:**
```
  ⚠️  WARNING: Drone is STUCK (not moving toward target)
```
**Meaning:** Drone is armed, has error > 0.5m, but velocity is near zero
**Cause:** Velocity commands not being applied to physics

### ⚠️ **Velocity Mismatch:**
```
  ⚠️  Velocity mismatch! Command not applied properly
```
**Meaning:** PID calculated velocity but physics received something different
**Cause:** `set_velocities()` not working or being overridden

### ⚠️ **Altitude Error:**
```
  ⚠️  WARNING: Large altitude error (1.50m)
```
**Meaning:** Drone not reaching target altitude
**Cause:** PID gains too weak or velocity not being applied

---

## What to Look For

### **Test 1: Is PID Working?**
Check if `PID Output` values are non-zero when there's an error:
```
  Error      : (  0.000,   0.000,   1.500) = 1.500m
  PID Output : (  0.000,   0.000,   1.200) m/s  ← Should be NON-ZERO!
```
✅ If PID Output is non-zero → PID is working
❌ If PID Output is zero with large error → PID broken

### **Test 2: Is Velocity Being Applied?**
Check if `Applied Vel` matches `PID Output`:
```
  PID Output : (  0.000,   0.000,   1.200) m/s
  Applied Vel: (  0.000,   0.000,   1.200) m/s  ← Should MATCH!
  ✅ Velocity command APPLIED correctly
```
✅ If they match → `set_velocities()` is working
❌ If they don't match → Physics engine rejecting commands

### **Test 3: Is Drone Actually Moving?**
Check if `Velocity` (current sensor reading) changes:
```
Frame 60:  Velocity: (0.000, 0.000, 0.000)
Frame 120: Velocity: (0.000, 0.000, 0.500)  ← Should INCREASE!
Frame 180: Velocity: (0.000, 0.000, 1.000)
```
✅ If velocity increases → Drone responding to commands
❌ If velocity stays zero → **ROOT CAUSE IDENTIFIED**

### **Test 4: Is Position Changing?**
Check if `Position` moves toward `Target Pos`:
```
Frame 60:  Position: (0.000, 0.000, 2.500)  Error: 1.500m
Frame 120: Position: (0.000, 0.000, 2.800)  Error: 1.200m  ← Error decreasing!
Frame 180: Position: (0.000, 0.000, 3.200)  Error: 0.800m
```
✅ If error decreases → Full control loop working
❌ If position doesn't change → Velocity not affecting physics

---

## Common Failure Patterns

### **Pattern 1: "Drone Falls While Armed"**
```
Frame 0:   Position: (0.000, 0.000, 2.500)  Mode: IDLE
[ARM EVENT] Drone ARMED
Frame 60:  Position: (0.000, 0.000, 2.300)  Mode: IDLE  ← Fell!
Frame 120: Position: (0.000, 0.000, 2.100)
```
**Diagnosis:** Gravity active but no upward velocity being applied
**Fix:** Check if IDLE mode sets velocities or if disarmed code runs instead

### **Pattern 2: "Drone Doesn't Move on Takeoff"**
```
[TAKEOFF] Current altitude: 2.50m → Target: 4.00m
Frame 60:  Error: (0.000, 0.000, 1.500)  PID Output: (0.000, 0.000, 1.200)
Frame 120: Position: (0.000, 0.000, 2.500)  ← NO CHANGE!
  ⚠️  WARNING: Drone is STUCK (not moving toward target)
```
**Diagnosis:** PID working, but velocity not applied to physics
**Fix:** Check `set_velocities()` implementation

### **Pattern 3: "Velocity Mismatch"**
```
  PID Output : (  0.000,   0.000,   1.200) m/s
  Applied Vel: (  0.000,   0.000,   0.000) m/s
  ⚠️  Velocity mismatch! Command not applied properly
```
**Diagnosis:** `set_velocities()` called but physics rejects it
**Fix:** Check if ArticulationView is modifying root vs joint velocities

---

## Testing Commands

### **Start Simulation:**
```bash
cd ~/ResQoUnity
./start_simulation.sh go2 1 flat office
```

### **Watch Logs in Real-Time:**
The simulation terminal will now show:
- Initialization logs
- Per-second frame updates  
- Warnings and errors

### **Run Test:**
```bash
python3 test_drone_forces.py
```

### **Manual Commands:**
```bash
# Arm
ros2 service call /drone/arm std_srvs/srv/SetBool "data: true"

# Takeoff (watch logs for 10 seconds)
ros2 service call /drone/takeoff std_srvs/srv/Trigger
```

---

## Expected Output (Working System)

```
[INIT] Drone spawned at position: (0.000, 0.000, 2.500)
[ARM EVENT] Drone ARMED
[TAKEOFF] Current altitude: 2.50m → Target: 4.00m

[FRAME 60]
  Position   : (  0.000,   0.000,   2.500)
  Error      : (  0.000,   0.000,   1.500)
  PID Output : (  0.000,   0.000,   1.200)
  Applied Vel: (  0.000,   0.000,   1.200)
  ✅ Velocity command APPLIED correctly

[FRAME 120]
  Position   : (  0.000,   0.000,   2.800)  ← MOVED UP!
  Velocity   : (  0.000,   0.000,   0.600)  ← HAS VELOCITY!
  Error      : (  0.000,   0.000,   1.200)  ← ERROR DECREASING!
  ✅ Velocity command APPLIED correctly

[FRAME 240]
  Position   : (  0.000,   0.000,   3.900)
  Error      : (  0.000,   0.000,   0.100)
  Arrived at target!
```

---

## Next Steps

1. **Run the simulation**
2. **Copy the frame logs** from simulation terminal
3. **Look for the patterns above** to identify the exact failure point
4. **Share the logs** and we can pinpoint the exact issue

The logs will tell us definitively:
- ✅ Is PID calculating velocities?
- ✅ Are velocities being sent to physics?
- ✅ Is physics engine accepting them?
- ✅ Is the drone actually moving?

**No more guessing - the logs show everything!** 🎯

