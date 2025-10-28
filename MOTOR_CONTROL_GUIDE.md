# Motor-Based Drone Control Implementation Guide

## Overview

The drone control system has been upgraded from **direct velocity bypass** to **proper motor control through Isaac Sim's physics engine**. The system now uses a cascade controller that outputs motor commands to the 4 rotors.

## What Changed

### Before (Velocity Bypass)
- Direct velocity teleportation using `robot.write_root_velocity_to_sim()`
- No motor physics involved
- Instant response, no dynamics

### After (Motor Control)
- Motor commands sent through action manager
- Full physics simulation with rotor thrust
- Attitude control via PID controllers
- Realistic flight dynamics

## Architecture

```
Position Target → Velocity PID (outer loop) → Desired velocities
                                                    ↓
                              Desired pitch/roll ← velocity-to-attitude
                                                    ↓
                              Attitude PID (inner loop) → Roll/pitch rates
                                                    ↓
                              Motor Mixer → [M1, M2, M3, M4] commands
                                                    ↓
                              Action Manager → Isaac Sim Physics → Drone flies
```

## Motor Configuration

**Crazyflie X-frame:**
```
    M4 (FL)         M1 (FR)
         \    ^    /
          \   |   /
            \ | /
              X
            / | \
          /   |   \
         /    v    \
    M3 (RL)         M2 (RR)
```

- **M1** (Front-Right): `thrust + roll - pitch + yaw`
- **M2** (Rear-Right): `thrust + roll + pitch - yaw`
- **M3** (Rear-Left): `thrust - roll + pitch + yaw`
- **M4** (Front-Left): `thrust - roll - pitch - yaw`

## Flight Modes

All high-level flight modes still work:

1. **VELOCITY** - Keyboard/cmd_vel control (converts velocities to motor commands)
2. **POSITION** - Autonomous waypoint navigation
3. **ALTITUDE_HOLD** - Maintain altitude, XY manual
4. **LOITER** - Hold current position
5. **LANDING** - Controlled descent
6. **EMERGENCY** - All motors off

## Files Modified

### 1. drone_controller.py (~110 lines added)
- Added attitude PID controllers (roll, pitch, yaw_rate)
- Added `_quat_to_euler()` method for attitude tracking
- Added `_motor_mixer()` for X-frame motor mixing
- Replaced `compute_velocity_command()` with `compute_motor_command()`
- Enhanced ground detection for landing

### 2. omniverse_sim.py (~60 lines modified)
- Removed `write_root_velocity_to_sim()` velocity bypass
- Added motor command computation loop
- Motor commands sent through action manager
- Updated controller initialization with attitude PID parameters
- Added numpy import

### 3. custom_rl_env.py
- No changes needed (action space already configured for 4 motors)

## Controller Parameters

```python
DroneController(
    # Position control (outer loop)
    kp_pos=0.8,      # Proportional gain for XY position
    ki_pos=0.01,     # Integral gain (anti-windup)
    kd_pos=0.4,      # Derivative gain (damping)
    
    # Altitude control
    kp_alt=1.5,      # Proportional gain for Z position
    ki_alt=0.05,     # Integral gain
    kd_alt=0.8,      # Derivative gain
    
    # Attitude control (inner loop) - NEW
    kp_att=2.0,      # Roll/pitch angle correction
    ki_att=0.0,      # Usually zero for attitude
    kd_att=0.5,      # Damping for oscillations
    
    # Yaw rate control - NEW
    kp_yaw=1.0,      # Yaw rate tracking
    ki_yaw=0.0,
    kd_yaw=0.2,
    
    # Limits
    max_vel=2.0,            # Max XY velocity (m/s)
    max_climb_rate=1.5,     # Max Z velocity (m/s)
    max_angle_rate=2.0,     # Max roll/pitch rate (rad/s) - NEW
    max_yaw_rate=1.0,       # Max yaw rate (rad/s) - NEW
    
    # Motor parameters - NEW
    hover_thrust=0.55       # Thrust to hover (0-1, tune this!)
)
```

## Testing Procedure

### Test 1: Hover Test (Tune hover_thrust)

**Goal:** Find the correct hover thrust value.

```bash
# Terminal 1: Start simulation
./start_simulation.sh drone 1 flat office

# Terminal 2: Arm and hover
cd ~/ResQoUnity/go2_omniverse_ws && source install/setup.bash

# Arm the drone
ros2 service call /robot0/arm std_srvs/srv/SetBool "{data: true}"

# Set to LOITER mode at current position (1m altitude)
ros2 topic pub --once /robot0/cmd_position geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'odom'}, pose: {position: {x: 0.0, y: 0.0, z: 1.0}}}"
```

**Expected:** Drone should hover at 1m altitude with minimal drift (±0.2m).

**If drone sinks:**
- Increase `hover_thrust` (try 0.60, 0.65)
- Restart simulation to apply

**If drone rises:**
- Decrease `hover_thrust` (try 0.50, 0.45)

### Test 2: Position Control

**Goal:** Test autonomous waypoint navigation.

```bash
# After Test 1 succeeds, send waypoint
ros2 topic pub --once /robot0/cmd_position geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'odom'}, pose: {position: {x: 2.0, y: 1.0, z: 1.5}}}"
```

**Expected:** Drone flies smoothly to (2, 1, 1.5) without overshooting.

**If oscillates:**
- Reduce `kp_pos` (try 0.6, 0.5)
- Increase `kd_pos` (try 0.6, 0.8)

**If too slow:**
- Increase `kp_pos` (try 1.0, 1.2)

### Test 3: Velocity Mode (Keyboard)

**Goal:** Test manual control via keyboard.

```bash
# Arm the drone (if not already armed)
ros2 service call /robot0/arm std_srvs/srv/SetBool "{data: true}"

# In Isaac Sim window, press keys:
# W - Move forward
# S - Move backward
# A - Move left
# D - Move right
# T - Altitude up
# G - Altitude down
# Q - Rotate left
# E - Rotate right
```

**Expected:** Smooth response to commands, no jerky motion.

**If too sensitive:**
- Reduce velocity-to-attitude gains in `omniverse_sim.py` line 588-590:
  ```python
  desired_pitch = -vx * 0.2  # Was 0.3
  desired_roll = vy * 0.2    # Was 0.3
  ```

**If too sluggish:**
- Increase gains (try 0.4, 0.5)

### Test 4: cmd_vel Control

**Goal:** Test programmatic velocity control.

```bash
# Move forward
ros2 topic pub --once /robot0/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {z: 0.0}}"

# Climb and move
ros2 topic pub --once /robot0/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 1.0}, angular: {z: 0.0}}"

# Stop
ros2 topic pub --once /robot0/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
```

### Test 5: Takeoff and Land

**Goal:** Test autonomous takeoff/landing services.

```bash
# Arm
ros2 service call /robot0/arm std_srvs/srv/SetBool "{data: true}"

# Takeoff to 1.5m
ros2 service call /robot0/takeoff std_srvs/srv/Trigger

# Wait 10 seconds...

# Land
ros2 service call /robot0/land std_srvs/srv/Trigger
```

**Expected:** Smooth takeoff, hover, then controlled descent.

### Test 6: SLAM Navigation

**Goal:** Verify motor control works with SLAM.

```bash
# Terminal 1: Simulation
./start_simulation.sh drone 1 flat office

# Terminal 2: SLAM
cd ~/ResQoUnity/go2_omniverse_ws && source install/setup.bash
ros2 launch go2_navigation drone_slam_launch.py \
    robot_name:=robot0 \
    flight_altitude:=1.5

# Terminal 3: RViz
rviz2
# Add Map: /map
# Add LaserScan: /robot0/scan
# Set Fixed Frame: odom

# Terminal 4: Arm and fly
ros2 service call /robot0/arm std_srvs/srv/SetBool "{data: true}"
ros2 service call /robot0/takeoff std_srvs/srv/Trigger

# Use keyboard (in Isaac Sim) to fly around and build map
```

## Tuning Guide

### Problem: Drone falls to ground
**Solution:** Increase `hover_thrust`
```python
hover_thrust=0.60  # Try 0.55, 0.60, 0.65
```

### Problem: Oscillations in hover
**Solution:** Reduce attitude gains or increase damping
```python
kp_att=1.5,  # Was 2.0
kd_att=0.7,  # Was 0.5
```

### Problem: Slow response to position commands
**Solution:** Increase position gains
```python
kp_pos=1.0,  # Was 0.8
kp_alt=2.0,  # Was 1.5
```

### Problem: Overshoot waypoints
**Solution:** Reduce position gains, increase damping
```python
kp_pos=0.6,  # Was 0.8
kd_pos=0.6,  # Was 0.4
```

### Problem: Drifts sideways
**Solution:** Tune velocity-to-attitude mapping
```python
# In omniverse_sim.py, line 588-589
desired_pitch = -vx * 0.25  # Tune this
desired_roll = vy * 0.25    # Tune this
```

### Problem: Motors saturate (all 0 or all 1)
**Solution:** Reduce overall thrust or attitude rates
```python
hover_thrust=0.50,        # Reduce if too high
max_angle_rate=1.5,       # Was 2.0
```

## Success Criteria

- ✅ Drone hovers stably (±0.2m drift over 10 seconds)
- ✅ Position control reaches target (±0.5m accuracy)
- ✅ Velocity mode responds smoothly to cmd_vel
- ✅ Keyboard control feels natural
- ✅ Takeoff/land services work reliably
- ✅ SLAM mapping works during flight
- ✅ No velocity bypass used (confirmed by physics simulation)

## Troubleshooting

### Drone spins uncontrollably
- Motor mixing might be wrong
- Check motor order matches Crazyflie X-frame
- Try setting `kp_yaw=0.5` (lower yaw gain)

### Drone flips over
- Attitude control too aggressive
- Reduce `kp_att` to 1.0 or lower
- Check quaternion to Euler conversion

### No response to commands
- Check if armed: `ros2 service call /robot0/arm std_srvs/srv/SetBool "{data: true}"`
- Check mode: Should be in VELOCITY or POSITION
- Verify motor commands are non-zero (add print statements)

### Motors don't spin
- Check action space: Should be (num_envs, 4)
- Verify actuator configuration in `robots/quadcopter/config.py`
- Check if actions are being sent to env.step()

## Advanced Tuning

For better performance, consider:

1. **Separate XY and Z gains** (currently they share position gains)
2. **Add velocity feedforward** to reduce lag
3. **Implement acceleration limits** for smoother flight
4. **Add wind compensation** (if simulating wind)
5. **Tune PID gains per axis** (X, Y, Z separately)

## Next Steps

Once basic flight works:

1. Train RL policy for specific tasks (if needed)
2. Implement path planning for navigation
3. Add obstacle avoidance
4. Multi-drone coordination
5. Advanced SLAM with loop closure

## Notes

- This implementation uses **small-angle approximation** (pitch ≈ velocity)
- Good for velocities < 3 m/s and angles < 30°
- For aggressive maneuvers, need full attitude quaternion control
- Motor mixing assumes symmetric X-frame (Crazyflie configuration)

