# ResQoUnity: Search and Rescue Robotics Simulation

> A simulation platform for autonomous search and rescue operations using ground and aerial robots.

---

## Project Overview

ResQoUnity is a digital twin simulation system designed for developing and testing search and rescue (SAR) robotics. The platform combines:

- **Unitree Go2** — A quadruped ground robot for terrain navigation and victim detection
- **Crazyflie Quadcopter** — An aerial drone for reconnaissance and area mapping

Both robots operate within NVIDIA Isaac Sim, providing realistic physics, sensor simulation, and ROS2 integration for algorithm development.

---

## System Architecture

```
┌──────────────────────────────────────────────────────────────────────────┐
│                        ResQoUnity Simulation                             │
├──────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│   ┌────────────────────┐          ┌────────────────────┐                │
│   │    Unitree Go2     │          │   Crazyflie Drone  │                │
│   │   (Ground Robot)   │          │   (Aerial Robot)   │                │
│   ├────────────────────┤          ├────────────────────┤                │
│   │ • 4-legged locomotion        │ • Quadcopter flight │                │
│   │ • Rough terrain capable      │ • Aerial surveying  │                │
│   │ • Camera + LiDAR             │ • Camera     │                │
│   │ • RL-based walking           │ • PID flight control│                │
│   └──────────┬─────────┘          └──────────┬─────────┘                │
│              │                               │                           │
│              └───────────┬───────────────────┘                           │
│                          │                                               │
│                          ▼                                               │
│   ┌──────────────────────────────────────────────────────────────────┐  │
│   │                         ROS2 Bridge                               │  │
│   │  • Odometry, IMU, Joint States                                   │  │
│   │  • Go2: Camera + LiDAR point clouds                              │  │
│   │  • Drone: Bottom camera stream                                   │  │
│   │  • Velocity commands, Position setpoints                         │  │
│   │  • Drone services (arm, takeoff, land)                           │  │
│   └──────────────────────────────────────────────────────────────────┘  │
│                          │                                               │
│                          ▼                                               │
│   ┌──────────────────────────────────────────────────────────────────┐  │
│   │                    External Systems                               │  │
│   │  • Nav2 (path planning)                                          │  │
│   │  • SLAM (mapping)                                                 │  │
│   │  • Mission planner                                                │  │
│   │  • Victim detection (future)                                      │  │
│   └──────────────────────────────────────────────────────────────────┘  │
│                                                                          │
└──────────────────────────────────────────────────────────────────────────┘
```

---

## Robot Specifications

### Unitree Go2 (Ground Robot)

| Attribute | Value |
|-----------|-------|
| Type | Quadruped (4-legged) |
| Degrees of Freedom | 12 joints (3 per leg) |
| Control Method | Reinforcement Learning (PPO) |
| Sensors | RGB Camera, RTX LiDAR, IMU |
| Capabilities | Rough terrain traversal, obstacle negotiation |

**Control System:**
The Go2 uses a trained neural network (PPO policy) that converts sensor observations into joint commands. The robot can walk, trot, and navigate uneven terrain autonomously while following velocity commands.

```
User Command (velocity) → RL Policy → Joint Targets → Physics → Movement
```

### Crazyflie Drone (Aerial Robot)

| Attribute | Value |
|-----------|-------|
| Type | Quadcopter |
| Degrees of Freedom | 4 rotors |
| Control Method | PID + Force-based velocity |
| Sensors | RGB Camera, IMU |
| Capabilities | Hover, position hold, waypoint navigation |

**Control System:**
The drone uses classical PID controllers to maintain position and follow velocity commands. Forces are calculated based on desired velocities and applied directly to achieve flight.

```
User Command → PID Controller → Force Calculation → Velocity → Movement
```

**Flight Modes:**

| Mode | Description |
|------|-------------|
| `VELOCITY` | Direct velocity control via cmd_vel |
| `POSITION` | Autonomous flight to target coordinates |
| `LOITER` | Hold current position (hover) |
| `ALTITUDE_HOLD` | Maintain altitude, free horizontal movement |
| `LANDING` | Controlled descent sequence |

---

## Sensor Suite

### Go2 Sensors

| Sensor | ROS2 Topic | Description |
|--------|------------|-------------|
| RGB Camera (front) | `/robot0/front_cam/image_raw` | Forward-facing, 640×480 |
| RTX LiDAR | `/robot0/point_cloud2` | 360° scanning, SLAM-capable |
| IMU | `/robot0/imu` | Orientation + velocity |
| Odometry | `/robot0/odom` | Position in world frame |
| Joint States | `/robot0/joint_states` | 12 joint positions |
| Foot Contact | `/robot0/go2_states` | Ground contact forces |

### Drone Sensors

| Sensor | ROS2 Topic | Description |
|--------|------------|-------------|
| RGB Camera (bottom) | `/drone/bottom_cam/image_raw` | Downward-facing, 640×480 |
| IMU | `/drone/imu` | Orientation + velocity |
| Odometry | `/drone/odom` | Position in world frame |
| Joint States | `/drone/joint_states` | Rotor positions |

### Sensor Details

**Go2 LiDAR (Unitree L1):**
- 360° horizontal scanning
- RTX ray-traced point cloud
- Suitable for SLAM and obstacle detection

**Go2 Camera:**
- 640×480 resolution
- Forward-facing mount for navigation

**Drone Camera:**
- 640×480 resolution
- Bottom-facing mount for aerial observation
- Useful for victim detection and area mapping

---

## ROS2 Interface

### Control Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/robot{i}/cmd_vel` | `geometry_msgs/Twist` | Velocity command (vx, vy, yaw_rate) |
| `/drone/cmd_vel` | `geometry_msgs/Twist` | Drone velocity (vx, vy, vz, yaw) |
| `/drone/cmd_position` | `geometry_msgs/PoseStamped` | Drone position setpoint |

### Drone Services

| Service | Type | Description |
|---------|------|-------------|
| `/drone/arm` | `std_srvs/SetBool` | Arm/disarm motors |
| `/drone/takeoff` | `std_srvs/Trigger` | Take off to 1.5m altitude |
| `/drone/land` | `std_srvs/Trigger` | Initiate landing sequence |
| `/drone/emergency_stop` | `std_srvs/Trigger` | Immediate stop |

### Example Commands

```bash
# Move Go2 forward
ros2 topic pub /robot0/cmd_vel geometry_msgs/Twist "{linear: {x: 0.5, y: 0, z: 0}, angular: {z: 0}}"

# Arm the drone
ros2 service call /drone/arm std_srvs/srv/SetBool "{data: true}"

# Take off
ros2 service call /drone/takeoff std_srvs/srv/Trigger

# Send drone to position
ros2 topic pub /drone/cmd_position geometry_msgs/PoseStamped "{pose: {position: {x: 5.0, y: 3.0, z: 2.0}}}"

# Land the drone
ros2 service call /drone/land std_srvs/srv/Trigger
```

---

## Operating Mode

### Combined Ground + Aerial (SAR Mode)

```bash
./run_sim.sh --robot=go2 --robot_amount=1
```

When running the simulation, both robots spawn automatically:
- **Go2 ground robot** for close-range search and victim interaction
- **Companion drone** for aerial observation and wide-area coverage

This enables coordinated search patterns where the drone provides aerial perspective while the Go2 navigates terrain.

**ROS2 Namespaces:**
- Ground robot: `/robot0/*`
- Companion drone: `/drone/*`

---

## Terrain and Environments

### Terrain Types

| Type | Command | Description |
|------|---------|-------------|
| Rough | `--terrain=rough` | Procedurally generated uneven terrain |
| Flat | `--terrain=flat` | Flat ground plane |

### Custom Environments

| Environment | Command | Description |
|-------------|---------|-------------|
| Office | `--custom_env=office --terrain=flat` | Indoor office layout |
| Warehouse | `--custom_env=warehouse --terrain=flat` | Industrial warehouse |

*Note: Custom environments require downloading assets from the project's Google Drive.*

---

## Keyboard Controls

### Ground Robot (Go2)

| Key | Action |
|-----|--------|
| W | Move forward |
| S | Move backward |
| A | Strafe left |
| D | Strafe right |
| Q | Rotate left |
| E | Rotate right |

### Drone

| Key | Action |
|-----|--------|
| W | Move forward |
| S | Move backward |
| A | Strafe left |
| D | Strafe right |
| Q | Rotate left |
| E | Rotate right |
| T | Ascend |
| G | Descend |

---

## Technical Implementation

### Simulation Stack

| Layer | Technology | Purpose |
|-------|------------|---------|
| Physics | NVIDIA PhysX 5 | Rigid body dynamics, collisions |
| Rendering | RTX Ray Tracing | Realistic visuals, LiDAR simulation |
| Robot Framework | Isaac Lab (Orbit) | Environment setup, RL integration |
| RL Training | RSL-RL / PPO | Locomotion policy for Go2 |
| Communication | ROS2 Humble | Robot control and sensor streaming |

### Control Loop (60 Hz)

```
┌─────────────────────────────────────────────────────────────┐
│                    Each Simulation Step                      │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  1. Read sensor data (IMU, joints, cameras, Go2 LiDAR)      │
│                                                              │
│  2. Process commands (keyboard, ROS2)                        │
│                                                              │
│  3. Compute control:                                         │
│     • Go2: RL policy → joint targets                        │
│     • Drone: PID → forces → velocities                      │
│                                                              │
│  4. Apply actions to physics simulation                      │
│                                                              │
│  5. Step physics engine                                      │
│                                                              │
│  6. Publish sensor data to ROS2                              │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### Go2 Locomotion Details

The Go2's walking behavior is learned through reinforcement learning:

**Observations (inputs to policy):**
- Base linear velocity (3D)
- Base angular velocity (3D)
- Gravity vector in body frame (3D)
- Commanded velocity (3D)
- Joint positions (12)
- Joint velocities (12)
- Terrain heightmap (160 points)

**Actions (outputs from policy):**
- 12 joint position targets (3 per leg × 4 legs)

**Actuators:**
- PD controllers with tuned stiffness/damping
- Physics engine computes required torques

### Drone Flight Details

The drone uses force-based velocity control:

**PID Gains (configurable):**
- Position: Kp=1.2, Ki=0.05, Kd=0.3
- Altitude: Kp=2.0, Ki=0.15, Kd=0.6

**Force Calculation:**
```
velocity_error = desired_velocity - current_velocity
acceleration = Kp × velocity_error
force = mass × acceleration + gravity_compensation
new_velocity = integrate(force / mass)
```

---

## File Structure

```
ResQoUnity/
├── main.py                    # Entry point
├── omniverse_sim.py           # Main simulation loop
├── drone_controller.py        # Drone flight controller (PID)
├── custom_rl_env.py           # Environment configurations
├── ros2_bridge.py             # ROS2 publishers/subscribers
├── agent_cfg.py               # RL agent configurations
│
├── robots/
│   └── quadcopter/
│       └── config.py          # Drone configuration
│
├── logs/
│   └── rsl_rl/
│       └── unitree_go2_rough/ # Trained Go2 policy
│
└── run_sim.sh                 # Launch SAR simulation (Go2 + Drone)
```

---

## Current Capabilities

| Feature | Status | Notes |
|---------|--------|-------|
| Go2 locomotion | ✅ Working | Trained RL policy |
| Go2 rough terrain | ✅ Working | Height scanner enabled |
| Drone flight | ✅ Working | PID-based control |
| Drone position hold | ✅ Working | ~2mm drift over 5 seconds |
| ROS2 sensor streaming | ✅ Working | All sensors available |
| ROS2 velocity control | ✅ Working | Both robots |
| Drone waypoint navigation | ✅ Working | Via cmd_position |
| Multi-robot (Go2 + Drone) | ✅ Working | Automatic companion drone |
| SLAM integration | ⚠️ Documented | Requires Nav2 setup |
| Autonomous search patterns | 🔲 Planned | Mission planner needed |
| Victim detection | 🔲 Planned | CV model integration |

---

## Future Development

### Planned Features for SAR

1. **Autonomous Search Patterns**
   - Grid search for drone
   - Frontier exploration for Go2
   - Coordinated multi-robot coverage

2. **Victim Detection**
   - Thermal camera simulation
   - Human detection model integration
   - Alert and localization system

3. **Communication Relay**
   - Drone as communication hub
   - Extended range for Go2 operations

4. **Hazard Detection**
   - Structural instability assessment
   - Gas/smoke detection simulation

5. **Multi-Robot Coordination**
   - Multiple Go2 units
   - Drone swarm support
   - Task allocation system

---

## Quick Start

### Prerequisites
- Ubuntu 22.04
- NVIDIA Isaac Sim 2023.1.1
- Isaac Lab (Orbit) 0.3.0
- ROS2 Humble
- NVIDIA GPU with RTX support

### Launch Simulation

```bash
# Launch Go2 + companion drone (SAR mode)
./run_sim.sh
```

### Control via ROS2

```bash
# Terminal 1: Launch simulation
./run_sim.sh

# Terminal 2: Control Go2
ros2 topic pub /robot0/cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}}"

# Terminal 3: Control drone
ros2 service call /drone/arm std_srvs/srv/SetBool "{data: true}"
ros2 service call /drone/takeoff std_srvs/srv/Trigger
ros2 topic pub /drone/cmd_position geometry_msgs/PoseStamped "{pose: {position: {x: 2, y: 2, z: 3}}}"
```

---

## References

- [NVIDIA Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/)
- [Isaac Lab (Orbit) Documentation](https://isaac-orbit.github.io/)
- [Unitree Go2 Specifications](https://www.unitree.com/go2)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)

---

*ResQoUnity — Simulation platform for search and rescue robotics research*
