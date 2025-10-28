# Drone Integration Guide

## Overview

The ResQoUnity project now supports the **Crazyflie Quadcopter Drone** as a fully controllable robot type alongside the Unitree Go2 and G1 robots. The drone features complete ROS2 integration with sensor suite, keyboard controls, and cmd_vel topic support.

## Quick Start

### Running the Drone Simulation

```bash
# Basic drone simulation with 1 drone
./run_sim_drone.sh

# Multiple drones
./run_sim_drone.sh --num_robots 2

# Different terrain types
./run_sim_drone.sh --terrain rough

# Custom environment
./run_sim_drone.sh --env warehouse --terrain flat
```

### Manual Launch

```bash
python3 main.py --robot drone --robot_amount 1 --terrain flat --custom_env office
```

Alternative robot name:
```bash
python3 main.py --robot quadcopter --robot_amount 1
```

## Drone Controls

### Keyboard Controls

| Key | Action |
|-----|--------|
| W | Forward (positive X velocity) |
| S | Backward (negative X velocity) |
| A | Strafe Left (positive Y velocity) |
| D | Strafe Right (negative Y velocity) |
| Q | Rotate Left (positive yaw) |
| E | Rotate Right (negative yaw) |
| **T** | **Altitude Up (drone only)** |
| **G** | **Altitude Down (drone only)** |

### ROS2 cmd_vel Control

Subscribe to the drone's velocity commands:

```bash
# Single drone
ros2 topic pub /robot0/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

# Multiple drones (robot0, robot1, etc.)
ros2 topic pub /robot1/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**Note:** The linear Y component is used for vertical velocity (altitude) in drone mode, while Z is typically used for yaw control.

## ROS2 Topics

### Published Topics (per drone)

| Topic | Type | Description |
|-------|------|-------------|
| `/robot{N}/joint_states` | `sensor_msgs/JointState` | Motor joint positions (m1-m4) |
| `/robot{N}/odom` | `nav_msgs/Odometry` | Position, orientation, and velocities |
| `/robot{N}/imu` | `sensor_msgs/Imu` | IMU data (critical for flight control) |
| `/robot{N}/point_cloud2` | `sensor_msgs/PointCloud2` | Lidar point cloud data |
| `/robot{N}/front_cam/rgb` | `sensor_msgs/Image` | Front camera RGB stream |

### Subscribed Topics (per drone)

| Topic | Type | Description |
|-------|------|-------------|
| `/robot{N}/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |

### TF Frames

- `/odom` → `/robot{N}/base_link`: Drone odometry transform

## Drone Configuration

### USD Model

The drone uses the **Crazyflie cf2x.usd** model from Isaac Sim's asset library:
- **Path:** `Isaac/Robots/Crazyflie/cf2x.usd`
- **Joints:** 4 rotor motors (m1_joint, m2_joint, m3_joint, m4_joint)
- **Scale:** 5x enlarged for better visibility

### Environment Configuration

**File:** `custom_rl_env.py`

The `QuadcopterEnvCfg` class configures:
- Robot articulation from `QUADCOPTER_CFG`
- Action scale: 0.1 (smaller than ground robots for stable flight)
- Rewards optimized for aerial navigation:
  - Disabled feet_air_time (no feet!)
  - Penalties for tilting and vertical oscillation
  - Flat orientation reward for stable flight
- Termination on base contact (crash detection)

### Agent Configuration

**File:** `agent_cfg.py`

The `quadcopter_agent_cfg` uses a smaller neural network:
- **Hidden dims:** [256, 128, 64] (vs [512, 256, 128] for ground robots)
- **Reason:** Drones have simpler action space (4 rotors vs 12+ joints)
- **Experiment name:** `quadcopter_rough`

## Sensor Configuration

### Camera

- **Location:** Base/body, front-facing with slight downward tilt
- **Resolution:** 640x480
- **Update rate:** 10 Hz
- **Offset:** (0.1, 0.0, -0.05) from base

### Lidar

- **Type:** RTX Lidar (ray-traced)
- **Location:** Base/body
- **Configuration:** Unitree_L1 profile
- **Frequency:** 200 Hz
- **Use case:** Obstacle avoidance, altitude sensing

### IMU

- **Location:** Base link
- **Data:** Orientation, linear acceleration, angular velocity
- **Critical for:** Flight stabilization and control

## Architecture

### Key Files Modified

1. **`custom_rl_env.py`** - Added `QuadcopterEnvCfg` class
2. **`agent_cfg.py`** - Added `quadcopter_agent_cfg` dictionary
3. **`omniverse_sim.py`** - Added drone robot type support, keyboard controls
4. **`ros2.py` / `ros2_bridge.py`** - Extended sensor functions for drone
5. **`go2_interfaces/msg/DroneState.msg`** - New message type for drone telemetry

### File Structure

```
ResQoUnity/
├── robots/
│   └── quadcopter/
│       ├── __init__.py
│       └── config.py          # QUADCOPTER_CFG definition
├── go2_omniverse_ws/
│   └── src/
│       └── go2_interfaces/
│           └── msg/
│               └── DroneState.msg  # New drone message
├── run_sim_drone.sh           # Drone launch script
├── custom_rl_env.py          # QuadcopterEnvCfg
├── agent_cfg.py              # quadcopter_agent_cfg
└── ros2.py                   # ROS2 integration
```

## DroneState Message

**File:** `go2_interfaces/msg/DroneState.msg`

```
# Altitude above ground (meters)
float32 altitude

# Battery level (0.0 to 1.0)
float32 battery_level

# Motor speeds (4 motors)
float32[4] motor_speeds

# Flight mode
uint8 flight_mode

# Armed state
bool armed

# GPS coordinates
float64 latitude
float64 longitude

# Velocity in body frame
float32[3] velocity_body

# Barometric pressure
float32 pressure
```

## Training Notes

### Model Checkpoints

The drone looks for trained models in:
```
logs/rsl_rl/quadcopter_rough/[timestamp]/model_*.pt
```

If no checkpoint exists, the simulation will attempt to train from scratch.

### Terrain Compatibility

- **Flat terrain:** Recommended for initial testing and stable flight
- **Rough terrain:** Advanced testing, requires robust flight controller
- **Custom environments:** Office and warehouse work well with obstacles

## Differences from Ground Robots

| Feature | Ground Robots (Go2/G1) | Quadcopter Drone |
|---------|----------------------|------------------|
| Joints | 12+ leg joints | 4 rotor motors |
| Movement | Ground locomotion | 3D flight |
| Sensors | Feet contact sensors | Primarily IMU-based |
| Control | Walk/run gaits | Thrust vectoring |
| Termination | Body contact | Base crash |
| Action scale | 0.25-0.5 | 0.1 |

## Troubleshooting

### Drone Not Spawning

1. Check USD asset path availability
2. Verify `QuadcopterEnvCfg` is imported correctly
3. Check console for errors related to QUADCOPTER_CFG

### Unstable Flight

1. Reduce action scale in `QuadcopterEnvCfg`
2. Increase orientation penalties in rewards
3. Use flat terrain for initial testing

### ROS2 Topics Not Publishing

1. Ensure ROS2 bridge is enabled
2. Check robot type is "drone" or "quadcopter"
3. Verify sensor attachment points in USD model

### Camera/Lidar Not Working

1. Check prim paths match USD structure
2. Verify sensor offsets are correct
3. Ensure base link exists in drone model

## Future Enhancements

Potential improvements for drone integration:

- [ ] Autonomous waypoint navigation
- [ ] Swarm behavior for multiple drones
- [ ] Dynamic obstacle avoidance
- [ ] Battery simulation and recharging
- [ ] GPS-based navigation
- [ ] Formation flying
- [ ] Custom drone USD models
- [ ] Advanced flight modes (hover, orbit, follow)

## Building ROS2 Messages

After modifying DroneState.msg:

```bash
cd go2_omniverse_ws
colcon build --packages-select go2_interfaces
source install/setup.bash
```

## SLAM Support

**Yes! The drone fully supports SLAM for aerial mapping.** 🗺️

### Quick SLAM Start

```bash
# Terminal 1: Launch drone
./run_sim_drone.sh

# Terminal 2: Launch drone-optimized SLAM
ros2 launch go2_navigation drone_slam_launch.py \
  robot_name:=robot0 \
  flight_altitude:=2.0
```

### SLAM Features

- ✅ 2D occupancy grid mapping at flight altitude
- ✅ Configurable altitude and slice thickness
- ✅ Optimized for aerial navigation
- ✅ Multi-drone SLAM support
- ✅ Map saving and reuse

**For complete SLAM documentation, see: [DRONE_SLAM_GUIDE.md](DRONE_SLAM_GUIDE.md)**

## Examples

### View Drone Topics

```bash
# List all drone topics
ros2 topic list | grep robot0

# Echo odometry
ros2 topic echo /robot0/odom

# Echo IMU
ros2 topic echo /robot0/imu

# View point cloud in RViz
rviz2
```

### Control Multiple Drones

```bash
# Launch 3 drones
./run_sim_drone.sh --num_robots 3

# In separate terminals, control each:
ros2 topic pub /robot0/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}"
ros2 topic pub /robot1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.5}}"
ros2 topic pub /robot2/cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.5}}"
```

## Credits

Drone integration based on Isaac Sim's Crazyflie cf2x USD model.
Configuration adapted from Unitree Go2/G1 robot setups.

---

**For general ResQoUnity documentation, see the main README.md**

