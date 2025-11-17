# Mixed Robots Configuration Guide

## Overview

The simulation now supports **mixed robot scenarios**: ground robots (Go2, G1) with a companion drone.

---

## 🤖 Configuration Matrix

| Command | Ground Robots | Drones | Control Method |
|---------|---------------|--------|----------------|
| `./start_simulation.sh` | 1 Go2 | 1 World Drone | Go2: Keyboard (WASD), Drone: ROS2 |
| `./start_simulation.sh go2 2` | 2 Go2 | 1 World Drone | Go2s: Keyboard, Drone: ROS2 |
| `./start_simulation.sh g1 1` | 1 G1 | 1 World Drone | G1: Keyboard, Drone: ROS2 |
| `./start_simulation.sh drone 1` | None | 1 Env Drone | Drone: Keyboard + ROS2 |
| `./start_simulation.sh drone 2` | None | 2 Env Drones | Drones: ROS2 only |

---

## 🎮 Control Methods

### Ground Robots (Go2/G1)
**Keyboard Controls:**
```
W - Forward
S - Backward
A - Strafe Left
D - Strafe Right
Q - Rotate Left
E - Rotate Right
```

### World Drone (when robot=go2 or g1)
**ROS2 Commands Only:**

```bash
# Arm the drone
ros2 service call /drone/arm std_srvs/srv/SetBool "{data: true}"

# Takeoff to 1.5m
ros2 service call /drone/takeoff std_srvs/srv/Trigger

# Velocity control
ros2 topic pub --once /drone/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {z: 0.0}}"

# Altitude control
ros2 topic pub --once /drone/cmd_altitude std_msgs/msg/Float32 "{data: 1.0}"

# Position control
ros2 topic pub --once /drone/cmd_position geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'odom'}, pose: {position: {x: 1.0, y: 0.5, z: 2.0}}}"

# Land
ros2 service call /drone/land std_srvs/srv/Trigger

# Emergency stop
ros2 service call /drone/emergency_stop std_srvs/srv/Trigger
```

### Environment Drones (when robot=drone)
**Keyboard + ROS2:**
- Keyboard works (WASDQE TG keys)
- ROS2 namespace: `/robot0/*`, `/robot1/*`, etc.
- Test script `test_drone_diagnostics.py` works ✅

---

## 🔧 Technical Details

### Architecture

**Two Drone Systems:**

1. **Environment Drones** (`robot=drone`):
   - Spawned by Isaac Lab environment at `/World/envs/env_X/Robot`
   - Managed by environment physics step
   - ROS2 namespace: `/robot0`, `/robot1`, etc.
   - Full keyboard control support

2. **World Drone** (`robot=go2|g1`):
   - Spawned manually at `/World/envs/env_0/Drone`
   - Independent physics management
   - ROS2 namespace: `/drone`
   - ROS2-only control (no keyboard)

### Why Two Systems?

**Historical Design:**
- Originally, drones and ground robots used different control architectures
- World drone was added as a "companion" to ground robots
- Environment drones support multi-agent scenarios

**Current State:**
- Both systems work, but have different namespaces
- Can't easily mix environment drones + world drone (would conflict)
- Logic: `if robot != "drone"` → spawn world drone

---

## 📝 Common Scenarios

### Scenario 1: Dog + Drone Interaction
```bash
# Start with 1 Go2 + 1 drone
./start_simulation.sh go2 1 flat office

# Terminal 1: Simulation running
# Terminal 2: Control drone
ros2 service call /drone/arm std_srvs/srv/SetBool "{data: true}"
ros2 service call /drone/takeoff std_srvs/srv/Trigger

# Terminal 3: Control Go2 with keyboard (focus on sim window, use WASD)
```

### Scenario 2: Multiple Dogs + Drone
```bash
# Start with 2 Go2 dogs + 1 drone
./start_simulation.sh go2 2 flat office

# Control:
# - Dog 0: WASDQE (primary keyboard)
# - Dog 1: IJKL UO (secondary keyboard)
# - Drone: ROS2 commands (/drone/*)
```

### Scenario 3: Pure Drone Testing
```bash
# Start with only drones (for diagnostic testing)
./start_simulation.sh drone 1 flat office

# Control:
# - Keyboard: WASDQE TG
# - ROS2: /robot0/*
# - Tests: test_drone_diagnostics.py works ✅
```

---

## 🧪 Testing

### Test World Drone (/drone namespace)

**Quick Test:**
```bash
# 1. Start mixed simulation
./start_simulation.sh go2 1

# 2. Check services available
ros2 service list | grep drone

# Expected:
# /drone/arm
# /drone/emergency_stop
# /drone/land
# /drone/takeoff

# 3. Arm and takeoff
ros2 service call /drone/arm std_srvs/srv/SetBool "{data: true}"
ros2 service call /drone/takeoff std_srvs/srv/Trigger

# 4. Monitor odometry
ros2 topic echo /drone/odom
```

### Test Environment Drones (/robot0 namespace)

```bash
# 1. Start pure drone mode
./start_simulation.sh drone 1

# 2. Run diagnostic test suite
python3 test_drone_diagnostics.py

# Expected: All 6 tests pass
```

---

## ⚠️ Limitations

### Current Constraints

1. **Keyboard control only works for environment robots:**
   - In `go2` mode: Keyboard controls Go2, NOT the world drone
   - In `drone` mode: Keyboard controls drones

2. **Different namespaces:**
   - World drone: `/drone/*`
   - Environment drones: `/robot0/*`, `/robot1/*`

3. **Test script compatibility:**
   - `test_drone_diagnostics.py` expects `/robot0/*` namespace
   - Only works in pure `drone` mode
   - Doesn't work with world drone (`/drone/*` namespace)

4. **Cannot mix environment + world drones:**
   - Running `drone` mode spawns ONLY environment drones
   - Running `go2` mode spawns ONLY world drone
   - No overlap to avoid conflicts

---

## 🎯 Best Practices

### For Development/Testing
Use **pure drone mode** (`robot=drone`):
```bash
./start_simulation.sh drone 1
```
- ✅ Keyboard control works
- ✅ Test scripts work
- ✅ Easier debugging

### For Multi-Robot Demos
Use **mixed mode** (`robot=go2`):
```bash
./start_simulation.sh go2 2
```
- ✅ See robots + drone together
- ✅ Demonstrate coordination
- ⚠️ Need ROS2 for drone control

### For Production
Consider implementing proper multi-robot environment (future work):
- Unified namespace
- Consistent control interface
- Single test framework

---

## 🚀 Quick Start

**Default setup (1 dog + 1 drone):**
```bash
# 1. Start simulation (defaults to go2 with 1 robot)
./start_simulation.sh

# 2. Control Go2 dog
# Focus on simulation window, use WASD keys

# 3. Control drone (in separate terminal)
source ~/ResQoUnity/IsaacSim-ros_workspaces/humble_ws/install/setup.bash
source ~/ResQoUnity/go2_omniverse_ws/install/setup.bash

ros2 service call /drone/arm std_srvs/srv/SetBool "{data: true}"
ros2 service call /drone/takeoff std_srvs/srv/Trigger

# Move drone forward
ros2 topic pub --once /drone/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0}}"
```

---

## 📚 Related Documentation

- `DRONE_CALIBRATION_GUIDE.md` - Physics calibration
- `SUCCESS_SUMMARY.md` - Drone control implementation
- `DRONE_SPAWN_ANALYSIS.md` - Troubleshooting spawn issues

---

**Happy Flying! 🚁🐕**

