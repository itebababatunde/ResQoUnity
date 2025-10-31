# Drone Implementation Complete ✅

## Summary

Successfully implemented a separate Crazyflie drone entity that operates alongside Go2 robots in the same Isaac Sim environment with full ROS2 control.

## Architecture

### Spawning Strategy: World-Level Drone (Option B)
- **Go2 Robots**: Managed by env system at `/World/envs/env_0/Robot` → ROS2 namespace `/robot0`
- **Drone**: Manually spawned at `/World/Drone` → ROS2 namespace `/drone`
- **Collaboration**: Both in same `/World/` space → same viewport, shared coordinates

### Why This Approach?
✅ Simpler implementation (no scene system modifications)  
✅ Both entities visible in same viewport  
✅ Shared coordinate frame enables collaboration  
✅ Independent management (Go2 uses RL env, drone uses manual control)

## Files Modified

### 1. `custom_rl_env.py`
**Changes:**
- Added `world_drone_controller` - DroneController instance for `/World/Drone`
- Added `world_drone_command` - Velocity commands `[vx, vy, yaw_rate]`
- Added `world_drone_altitude` - Altitude command for Z-axis control

### 2. `omniverse_sim.py`
**Changes:**
- Imported `QUADCOPTER_CFG` and `Articulation` from Isaac Orbit
- Added world drone callback functions (lines 342-470):
  - `world_drone_cmd_vel_cb()` - Velocity control
  - `world_drone_cmd_position_cb()` - Position waypoints
  - `world_drone_cmd_altitude_cb()` - Altitude hold
  - `world_drone_takeoff_cb()` - Takeoff service
  - `world_drone_land_cb()` - Landing service
  - `world_drone_emergency_stop_cb()` - Emergency stop
  - `world_drone_arm_cb()` - Arm/disarm service
- Updated `add_cmd_sub()` to accept `enable_world_drone` parameter
- Added ROS2 topics/services for `/drone` namespace (lines 500-513)
- Spawned drone as proper Articulation (lines 663-733):
  - Create drone with `QUADCOPTER_CFG`
  - Initialize DroneController with PID gains
  - Add bottom-facing camera (480x640, 90° down)
- Integrated drone into simulation loop (lines 816-861):
  - Read drone state every frame
  - Update DroneController
  - Apply motor commands
  - Write data to physics
- Added drone data publishing (lines 869-873)

### 3. `ros2_bridge.py`
**Changes:**
- Updated `RobotBaseNode.__init__()` to accept `enable_world_drone` parameter
- Added drone publishers: `/drone/odom`, `/drone/imu`, `/drone/joint_states`
- Added publishing methods:
  - `publish_drone_odom()` - Position, orientation, transforms
  - `publish_drone_imu()` - IMU data (accel, gyro, orientation)
  - `publish_drone_joints()` - Motor joint states

## ROS2 Interface

### Topics (Continuous Control)

#### Velocity Control
```bash
ros2 topic pub /drone/cmd_vel geometry_msgs/msg/Twist \
  "linear: {x: 1.0, y: 0.0, z: 0.5}
   angular: {z: 0.2}"
```
- `linear.x`: Forward velocity (m/s)
- `linear.y`: Strafe velocity (m/s)
- `linear.z`: Vertical velocity (m/s)
- `angular.z`: Yaw rate (rad/s)

#### Position Control (Waypoints)
```bash
ros2 topic pub /drone/cmd_position geometry_msgs/msg/PoseStamped \
  "pose: {position: {x: 5.0, y: 3.0, z: 2.0}}"
```

#### Altitude Hold
```bash
ros2 topic pub /drone/cmd_altitude std_msgs/msg/Float32 "data: 3.5"
```

### Services (Discrete Commands)

#### Arm Drone
```bash
ros2 service call /drone/arm std_srvs/srv/SetBool "data: true"
```

#### Takeoff
```bash
ros2 service call /drone/takeoff std_srvs/srv/Trigger
```
- Arms drone → lifts to 1.5m → holds position

#### Land
```bash
ros2 service call /drone/land std_srvs/srv/Trigger
```

#### Emergency Stop
```bash
ros2 service call /drone/emergency_stop std_srvs/srv/Trigger
```

### Published Topics (Feedback)

```bash
/drone/odom                    # nav_msgs/Odometry (position, velocity)
/drone/imu                     # sensor_msgs/Imu (acceleration, gyro, orientation)
/drone/joint_states            # sensor_msgs/JointState (motor positions)
```

## Testing Guide

### Step 1: Launch Simulation
```bash
cd ~/ResQoUnity
./start_simulation.sh go2 1 flat office
```

**Expected Output:**
```
[INFO] Spawning world-level drone at /World/Drone...
[INFO] Drone spawned successfully at /World/Drone
[INFO] Drone controller initialized in DISARMED mode
[INFO] Drone bottom camera initialized
[INFO] Adding /drone namespace ROS2 interface
[INFO] Drone control interface ready on /drone namespace
[INFO] Drone ROS2 publishers initialized on /drone namespace
```

### Step 2: Verify Drone Visible
- In Isaac Sim viewport, you should see:
  - Go2 robot on the ground (robot0)
  - Crazyflie drone hovering at 2.5m (scaled 5x for visibility)
- Press `F` to focus camera on drone/robot

### Step 3: Check ROS2 Topics
```bash
# Open new terminal
source /opt/ros/humble/setup.bash

# List drone topics
ros2 topic list | grep drone
```

**Expected Output:**
```
/drone/cmd_altitude
/drone/cmd_position
/drone/cmd_vel
/drone/imu
/drone/joint_states
/drone/odom
```

### Step 4: Check ROS2 Services
```bash
ros2 service list | grep drone
```

**Expected Output:**
```
/drone/arm
/drone/emergency_stop
/drone/land
/drone/takeoff
```

### Step 5: Arm Drone
```bash
ros2 service call /drone/arm std_srvs/srv/SetBool "data: true"
```

**Expected Output:**
```
success: True
message: 'Drone armed'
```

### Step 6: Takeoff Test
```bash
ros2 service call /drone/takeoff std_srvs/srv/Trigger
```

**Expected:**
- Drone lifts from 2.5m to ~4.0m (1.5m above initial position)
- Hovers stably at target altitude
- Console shows: `[Drone drone] Target position: (0.00, 0.00, 4.00)`

### Step 7: Position Control Test
```bash
# Move to waypoint
ros2 topic pub --once /drone/cmd_position geometry_msgs/msg/PoseStamped \
  "pose: {position: {x: 2.0, y: 1.0, z: 3.0}}"
```

**Expected:**
- Drone flies to (2.0, 1.0, 3.0)
- Smooth trajectory, arrives within ~0.5m tolerance
- Switches to LOITER mode on arrival

### Step 8: Velocity Control Test
```bash
# Fly forward at 0.5 m/s
ros2 topic pub /drone/cmd_vel geometry_msgs/msg/Twist \
  "linear: {x: 0.5, y: 0.0, z: 0.0}
   angular: {z: 0.0}" &

# Stop after 3 seconds
sleep 3 && ros2 topic pub --once /drone/cmd_vel geometry_msgs/msg/Twist \
  "linear: {x: 0.0, y: 0.0, z: 0.0}
   angular: {z: 0.0}"
```

### Step 9: Landing Test
```bash
ros2 service call /drone/land std_srvs/srv/Trigger
```

**Expected:**
- Drone descends slowly (~-0.3 m/s)
- Auto-disarms on ground contact

### Step 10: Monitor Telemetry
```bash
# Watch odometry
ros2 topic echo /drone/odom --field pose.pose.position

# Watch IMU
ros2 topic echo /drone/imu --field orientation

# Check publishing rate
ros2 topic hz /drone/odom
```

## Flight Modes

The DroneController supports multiple modes:

1. **DISARMED** - Motors off, no commands accepted
2. **IDLE** - Armed, on ground, ready for takeoff
3. **VELOCITY** - Direct velocity control via `cmd_vel`
4. **POSITION** - Position setpoint tracking with PID
5. **ALTITUDE_HOLD** - Maintain altitude, XY velocity control
6. **LOITER** - Hold current position (hover)
7. **LANDING** - Controlled descent to ground
8. **EMERGENCY** - Emergency stop, all velocities zeroed

## PID Tuning

Current PID gains (in `omniverse_sim.py` lines 682-698):

```python
kp_pos=0.3,      # Position control proportional gain
kd_pos=0.2,      # Position control derivative gain
kp_alt=0.8,      # Altitude control proportional gain
kd_alt=0.4,      # Altitude control derivative gain
kp_att=1.0,      # Attitude control proportional gain
kd_att=0.3,      # Attitude control derivative gain
hover_thrust=0.45 # Hover thrust baseline (0-1)
```

To tune:
1. Increase `kp_pos` for faster position response (but may overshoot)
2. Increase `kd_pos` for damping (reduces oscillations)
3. Adjust `hover_thrust` if drone drifts up/down when armed

## Troubleshooting

### Issue: Drone Not Spawning
**Check:**
```bash
# Verify QUADCOPTER_CFG is accessible
python3 -c "from robots.quadcopter.config import QUADCOPTER_CFG; print(QUADCOPTER_CFG)"
```

**Fix:** Ensure `robots/quadcopter/config.py` exists and is valid

### Issue: ROS2 Services Not Available
**Check:**
```bash
ros2 node list  # Should show /drone_control_node
```

**Fix:** Restart simulation, check for Python errors in terminal

### Issue: Drone Spinning/Unstable
**Causes:**
- PID gains too high
- Motor mixing incorrect
- Timestep issues

**Fix:**
1. Reduce `kp_att` and `kd_att`
2. Lower `hover_thrust` if climbing
3. Check simulation runs at 60 Hz

### Issue: Drone Ignoring Commands
**Check Mode:**
```bash
# Drone only accepts velocity commands in VELOCITY/IDLE modes
# Check console for: "[Drone] Ignoring cmd_vel in mode POSITION"
```

**Fix:**
- Call emergency stop to reset mode
- Re-arm drone
- Ensure drone is armed before sending commands

## Key Design Decisions

1. **Namespace Separation**: Go2 on `/robot{N}`, drone on `/drone` - prevents conflicts
2. **World-Level Spawning**: Drone outside env system - simpler, independent control
3. **Reuse DroneController**: Existing PID logic - no duplication
4. **Single Drone**: One drone for initial implementation - multi-drone can be added later
5. **Bottom Camera Only**: Minimal sensors as requested - extensible
6. **Position Control Primary**: Advanced waypoint navigation via `cmd_position` topic

## Next Steps (Optional Enhancements)

1. **Multi-Drone Support**: Spawn multiple drones at `/World/Drone0`, `/World/Drone1`, etc.
2. **Lidar for Drone**: Add lidar sensor for obstacle avoidance
3. **Formation Flight**: Coordinate multiple drones in formation
4. **Collaborative Tasks**: Drone guides Go2 to target location
5. **Camera Streaming**: Publish bottom camera images to ROS2
6. **Autonomous Navigation**: Integrate with Nav2 for path planning

## Files Summary

- ✅ `custom_rl_env.py` - Added drone state tracking
- ✅ `omniverse_sim.py` - Spawning, control, callbacks, ROS2 interface
- ✅ `ros2_bridge.py` - Publishing drone data
- ✅ `drone_controller.py` - Unchanged (already complete)
- ✅ `robots/quadcopter/config.py` - Unchanged (already complete)

## Verification Checklist

- [x] Drone spawns at `/World/Drone`
- [x] Drone controller initialized
- [x] Bottom camera added
- [x] ROS2 topics created (`/drone/*`)
- [x] ROS2 services created (`/drone/*`)
- [x] Publishing: odom, IMU, joint states
- [x] Velocity control integrated
- [x] Position control integrated
- [x] Takeoff/land services
- [x] Separate from Go2 namespace
- [x] Same viewport as Go2
- [x] No linter errors (only import warnings)

---

## Contact

For issues or questions:
1. Check console output for error messages
2. Verify ROS2 topics with `ros2 topic list`
3. Test with simple commands first (arm → takeoff → land)

**Implementation Status: COMPLETE** ✅

