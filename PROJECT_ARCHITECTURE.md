# ResQoUnity Project Architecture

> A comprehensive guide to understanding the digital twin simulation system for Unitree robots and Crazyflie drones built on NVIDIA Isaac Sim.

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [System Architecture](#2-system-architecture)
3. [Robot Types and USD Assets](#3-robot-types-and-usd-assets)
4. [Control Systems](#4-control-systems)
5. [Navigation and Locomotion](#5-navigation-and-locomotion)
6. [ROS2 Integration](#6-ros2-integration)
7. [Drone Systems](#7-drone-systems)
8. [Running the Simulation](#8-running-the-simulation)
9. [Current Gaps and Limitations](#9-current-gaps-and-limitations)
10. [File Reference](#10-file-reference)

---

## 1. Project Overview

### What This Project Does

This is a **digital twin simulation** system that allows you to:

- Simulate Unitree Go2 (quadruped dog) and G1 (humanoid) robots
- Simulate Crazyflie quadcopter drones
- Control robots via keyboard or ROS2 commands
- Stream sensor data (cameras, LiDAR, IMU) to ROS2
- Train and deploy reinforcement learning policies for locomotion
- Run mixed scenarios with ground robots and aerial drones

### Technology Stack

| Component | Technology |
|-----------|------------|
| Simulation Engine | NVIDIA Isaac Sim 2023.1.1 |
| Robot Framework | Isaac Lab (Orbit) 0.3.0 |
| RL Training | RSL-RL with PPO algorithm |
| Robot Communication | ROS2 Humble |
| Physics | PhysX 5 (GPU accelerated) |
| Assets | OpenUSD format |

---

## 2. System Architecture

### High-Level Data Flow

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           omniverse_sim.py                              │
│                        (Main Simulation Loop)                           │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐     │
│  │  Isaac Sim      │    │  Isaac Lab      │    │  RSL-RL         │     │
│  │  (Physics)      │◄──►│  (Environment)  │◄──►│  (RL Policy)    │     │
│  └─────────────────┘    └─────────────────┘    └─────────────────┘     │
│           │                      │                      │               │
│           ▼                      ▼                      ▼               │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                      Simulation Loop (60 Hz)                     │   │
│  │  1. Get observations (joint states, IMU, velocity)              │   │
│  │  2. Query policy (RL) or controller (PID)                       │   │
│  │  3. Apply actions (joint targets or velocities)                 │   │
│  │  4. Step physics                                                 │   │
│  │  5. Publish to ROS2                                              │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                    │                                    │
│                                    ▼                                    │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                        ROS2 Bridge                               │   │
│  │  Publishers: odom, imu, joint_states, point_cloud2, camera      │   │
│  │  Subscribers: cmd_vel, cmd_position                              │   │
│  │  Services: arm, takeoff, land, emergency_stop                    │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

### Entry Point

```
main.py
   └── run_sim()  [omniverse_sim.py]
          ├── AppLauncher (starts Isaac Sim)
          ├── gymnasium.make() (creates environment)
          ├── Load RL policy (if available)
          ├── Spawn robots via USD
          ├── Initialize ROS2 bridge
          └── Main loop (while simulation_app.is_running())
```

---

## 3. Robot Types and USD Assets

### What is USD?

**Universal Scene Description (USD)** is NVIDIA's file format for 3D assets. A robot's USD file contains:

- 3D meshes (visual geometry)
- Collision geometry
- Joint definitions (revolute, prismatic, etc.)
- Physics properties (mass, inertia, friction)
- Material properties

### Supported Robots

| Robot | Type | USD Source | Local Path |
|-------|------|------------|------------|
| **Unitree Go2** | Quadruped (4-legged dog) | Omniverse Nucleus (built-in) | N/A |
| **Unitree G1** | Humanoid (bipedal) | Local file | `./robots/g1/g1.usd` |
| **Crazyflie** | Quadcopter drone | Omniverse Nucleus | Remote URL |

### How Robots Are Imported

Each robot is defined as an `ArticulationCfg` that specifies:

```python
# Example: Quadcopter configuration (robots/quadcopter/config.py)
QUADCOPTER_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="http://...Isaac/Robots/Crazyflie/cf2x.usd",  # USD source
        scale=(5.0, 5.0, 5.0),                                  # Size multiplier
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            linear_damping=0.01,
            angular_damping=0.05,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 2.5),           # Spawn position [x, y, z]
        rot=(1.0, 0.0, 0.0, 0.0),      # Spawn orientation (quaternion)
        joint_pos={"m1_joint": 0.0, ...},  # Initial joint positions
    ),
    actuators={
        "rotors": ImplicitActuatorCfg(
            joint_names_expr=["m1_joint", "m2_joint", "m3_joint", "m4_joint"],
            effort_limit=100.0,
            velocity_limit=2000.0,
        ),
    },
)
```

### Robot Spawning in Isaac Lab

When the environment is created, robots spawn at:
- **Environment robots**: `/World/envs/env_{i}/Robot`
- **World drone** (companion): `/World/envs/env_0/Drone`

---

## 4. Control Systems

### Overview

The project uses two fundamentally different control approaches:

| Robot Type | Control Method | Why |
|------------|---------------|-----|
| Go2 / G1 | **RL Policy + Joint Position Control** | Walking is complex, learned behavior works best |
| Drone | **PID Controller + Force-Based Velocity** | Flight dynamics are well-understood, no RL needed |

---

### 4.1 Ground Robot Control (Go2 / G1)

#### The Control Pipeline

```
┌─────────────────────────────────────────────────────────────────────┐
│                    Every Physics Step (60 Hz)                        │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  STEP 1: OBSERVATIONS                                                │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │ • base_lin_vel: Linear velocity [vx, vy, vz]                │    │
│  │ • base_ang_vel: Angular velocity [wx, wy, wz]               │    │
│  │ • projected_gravity: Gravity vector in body frame           │    │
│  │ • velocity_commands: User input [vx, vy, yaw_rate]          │    │
│  │ • joint_pos: Current joint positions (12 values)            │    │
│  │ • joint_vel: Current joint velocities (12 values)           │    │
│  │ • height_scan: Terrain heightmap ahead of robot             │    │
│  └─────────────────────────────────────────────────────────────┘    │
│                              │                                       │
│                              ▼                                       │
│  STEP 2: NEURAL NETWORK (Trained PPO Policy)                        │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │  observations ──► [512] ──► [256] ──► [128] ──► actions     │    │
│  │                                                              │    │
│  │  The network learned to map sensor data to joint targets    │    │
│  │  through millions of simulated training episodes.           │    │
│  └─────────────────────────────────────────────────────────────┘    │
│                              │                                       │
│                              ▼                                       │
│  STEP 3: ACTIONS (12 Joint Position Targets)                        │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │  Front-Left:  [hip_yaw, hip_roll, hip_pitch, knee]          │    │
│  │  Front-Right: [hip_yaw, hip_roll, hip_pitch, knee]          │    │
│  │  Rear-Left:   [hip_yaw, hip_roll, hip_pitch, knee]          │    │
│  │  Rear-Right:  [hip_yaw, hip_roll, hip_pitch, knee]          │    │
│  └─────────────────────────────────────────────────────────────┘    │
│                              │                                       │
│                              ▼                                       │
│  STEP 4: ACTUATORS (PD Controllers in Physics Engine)               │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │  For each joint:                                             │    │
│  │    torque = stiffness × (target - current) - damping × vel  │    │
│  │                                                              │    │
│  │  The physics engine applies these torques to move joints    │    │
│  │  toward target positions smoothly.                          │    │
│  └─────────────────────────────────────────────────────────────┘    │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

#### What is Reinforcement Learning (RL)?

**RL** is a machine learning approach where an agent learns through trial and error:

1. **Agent** (robot) takes actions in an **environment** (simulation)
2. Gets **rewards** for good behavior (walking forward, staying balanced)
3. Gets **penalties** for bad behavior (falling, excessive energy use)
4. Over millions of trials, learns a **policy** (mapping from observations to actions)

#### What is PPO?

**Proximal Policy Optimization (PPO)** is the specific RL algorithm used. It's popular for robotics because:
- Stable training (doesn't diverge easily)
- Sample efficient (learns quickly)
- Works well with continuous action spaces (joint angles)

#### Trained Models

Pre-trained policies are stored in `logs/rsl_rl/`:

```
logs/rsl_rl/
├── unitree_go2_rough/
│   └── 2024-04-06_02-37-07/
│       └── model_7850.pt      # Go2 walking policy
└── g1_rough/
    └── 2024-06-03_21-09-07/
        └── model_2050.pt      # G1 walking policy
```

---

### 4.2 Drone Control (Crazyflie)

#### Why No RL for Drones?

| Walking (Complex) | Flying (Simpler) |
|------------------|------------------|
| 12+ joints to coordinate | 4 motors, direct force control |
| Ground contact dynamics | No contact (usually) |
| Balance is non-trivial | Inherently stable (with controller) |
| Terrain adaptation needed | Flies over obstacles |
| Gait selection (walk/trot/gallop) | Single flight mode |

Drone flight control is a **well-solved problem** with classical control theory (PID controllers).

#### The Drone Control Pipeline

```
┌─────────────────────────────────────────────────────────────────────┐
│                    DroneController (drone_controller.py)             │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  INPUT: Command (from keyboard, ROS2, or internal mode)             │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │  VELOCITY mode: [vx, vy, vz, yaw_rate] from cmd_vel         │    │
│  │  POSITION mode: [x, y, z] target from cmd_position          │    │
│  │  LOITER mode:   Hold current position                        │    │
│  │  LANDING mode:  Descend at fixed rate                        │    │
│  └─────────────────────────────────────────────────────────────┘    │
│                              │                                       │
│                              ▼                                       │
│  PID CONTROLLERS (compute desired velocities)                       │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │  pid_x: error_x → desired_vx                                 │    │
│  │  pid_y: error_y → desired_vy                                 │    │
│  │  pid_z: error_z → desired_vz                                 │    │
│  │                                                              │    │
│  │  output = Kp × error + Ki × ∫error + Kd × d(error)/dt       │    │
│  └─────────────────────────────────────────────────────────────┘    │
│                              │                                       │
│                              ▼                                       │
│  FORCE CALCULATION (physics-based)                                  │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │  velocity_error = desired_velocity - current_velocity        │    │
│  │  acceleration = Kp_accel × velocity_error                    │    │
│  │  force = mass × acceleration                                 │    │
│  │  force_z += mass × gravity  (gravity compensation)          │    │
│  └─────────────────────────────────────────────────────────────┘    │
│                              │                                       │
│                              ▼                                       │
│  VELOCITY INTEGRATION                                               │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │  new_velocity = current_velocity + (force/mass) × dt         │    │
│  │  new_velocity *= damping  (air resistance)                   │    │
│  │  robot.write_root_velocity_to_sim(new_velocity)             │    │
│  └─────────────────────────────────────────────────────────────┘    │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

#### Flight Modes

The `DroneController` supports multiple flight modes:

| Mode | Description | Trigger |
|------|-------------|---------|
| `DISARMED` | Motors off, no commands accepted | Default / after landing |
| `IDLE` | Armed, waiting for commands | After arming |
| `VELOCITY` | Direct velocity control | cmd_vel input |
| `POSITION` | Fly to target position | cmd_position input |
| `ALTITUDE_HOLD` | Maintain altitude, free XY movement | cmd_altitude input |
| `LOITER` | Hold current position (hover) | After reaching waypoint |
| `LANDING` | Controlled descent | land service call |
| `EMERGENCY` | Immediate stop | emergency_stop service |

#### Why Force-Based Control (Not Motor Thrust)?

The Crazyflie USD model has rotor joints that spin, but **they don't generate thrust physics**. This is a limitation of the USD asset.

**Ideal approach:**
```
Motor RPM → Propeller Aerodynamics → Thrust Force → Movement
```

**Current workaround:**
```
Desired Velocity → PID → Force Calculation → Direct Velocity Setting
```

This "hybrid physics-velocity" approach works but isn't physically accurate. For realistic drone simulation, consider NVIDIA's **OmniDrones** framework.

---

## 5. Navigation and Locomotion

### Important Distinction

The **RL policy does NOT handle navigation**. It handles **locomotion**.

```
┌─────────────────────────────────────────────────────────────────────┐
│                    NAVIGATION (High-Level)                          │
│                    NOT part of RL policy                            │
├─────────────────────────────────────────────────────────────────────┤
│  "Go to position (5, 3, 0)"                                         │
│       │                                                              │
│       ▼                                                              │
│  Path Planning (Nav2, SLAM, A*, etc.)                               │
│       │                                                              │
│       ▼                                                              │
│  Velocity Commands: cmd_vel [vx=0.5, vy=0.0, yaw_rate=0.1]         │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    LOCOMOTION (Low-Level)                           │
│                    THIS is what RL handles                          │
├─────────────────────────────────────────────────────────────────────┤
│  "Move forward at 0.5 m/s while turning slightly"                   │
│       │                                                              │
│       ▼                                                              │
│  RL Policy processes:                                                │
│  • Current velocity command                                          │
│  • Joint positions and velocities                                    │
│  • IMU data (balance)                                                │
│  • Terrain heightmap (upcoming ground)                              │
│       │                                                              │
│       ▼                                                              │
│  Outputs: 12 joint position targets                                 │
│  Result: Robot walks/trots/gallops toward commanded velocity        │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

### Height Scanner (Terrain Awareness)

The RL policy receives terrain information via a height scanner:

```
         Robot facing →
            │
       ┌────┴────┐
       │ Scanner │  (1.6m × 1.0m grid, 0.1m resolution = 160 points)
       └────┬────┘
            │
    ────────┼────────────────────────
      ╱     │     ╲        ╱
     ╱      │      ╲      ╱      Terrain
    ╱       │       ╲    ╱
```

This allows the policy to:
- Anticipate steps and obstacles
- Adjust gait for rough terrain
- Lift legs higher when needed

---

## 6. ROS2 Integration

### Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        ros2_bridge.py                               │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  RobotBaseNode (for each robot in simulation)                       │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │  Publishers:                                                 │    │
│  │    /robot{i}/joint_states  (JointState)                     │    │
│  │    /robot{i}/odom          (Odometry)                       │    │
│  │    /robot{i}/imu           (Imu)                            │    │
│  │    /robot{i}/point_cloud2  (PointCloud2)                    │    │
│  │    /robot{i}/go2_states    (Go2State - foot forces)         │    │
│  │                                                              │    │
│  │  Subscribers (in omniverse_sim.py):                         │    │
│  │    /robot{i}/cmd_vel       (Twist)                          │    │
│  │    /robot{i}/cmd_position  (PoseStamped) - drones only      │    │
│  │    /robot{i}/cmd_altitude  (Float32) - drones only          │    │
│  │                                                              │    │
│  │  Services (drones only):                                     │    │
│  │    /robot{i}/arm           (SetBool)                        │    │
│  │    /robot{i}/takeoff       (Trigger)                        │    │
│  │    /robot{i}/land          (Trigger)                        │    │
│  │    /robot{i}/emergency_stop (Trigger)                       │    │
│  └─────────────────────────────────────────────────────────────┘    │
│                                                                      │
│  World Drone Publishers (when enabled)                              │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │    /drone/odom             (Odometry)                       │    │
│  │    /drone/imu              (Imu)                            │    │
│  │    /drone/joint_states     (JointState)                     │    │
│  │    /drone/cmd_vel          (Twist)                          │    │
│  │    /drone/arm, takeoff, land, emergency_stop (Services)     │    │
│  └─────────────────────────────────────────────────────────────┘    │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

### Topic Reference

#### For Ground Robots (Go2/G1)

| Topic | Type | Description |
|-------|------|-------------|
| `/robot{i}/joint_states` | `JointState` | Current joint positions |
| `/robot{i}/odom` | `Odometry` | Position and orientation in world |
| `/robot{i}/imu` | `Imu` | Orientation, linear/angular velocity |
| `/robot{i}/point_cloud2` | `PointCloud2` | RTX LiDAR point cloud |
| `/robot{i}/go2_states` | `Go2State` | Foot contact forces |
| `/robot{i}/cmd_vel` | `Twist` | Velocity command input |

#### For Drones (Environment or World)

| Topic/Service | Type | Description |
|---------------|------|-------------|
| `/robot{i}/cmd_vel` or `/drone/cmd_vel` | `Twist` | Velocity command |
| `/robot{i}/cmd_position` or `/drone/cmd_position` | `PoseStamped` | Position setpoint |
| `/robot{i}/cmd_altitude` or `/drone/cmd_altitude` | `Float32` | Altitude setpoint |
| `/robot{i}/arm` or `/drone/arm` | `SetBool` | Arm/disarm motors |
| `/robot{i}/takeoff` or `/drone/takeoff` | `Trigger` | Take off to 1.5m |
| `/robot{i}/land` or `/drone/land` | `Trigger` | Begin landing |
| `/robot{i}/emergency_stop` or `/drone/emergency_stop` | `Trigger` | Emergency stop |

---

## 7. Drone Systems

### Two Drone Architectures

The project has **two different ways** to spawn drones:

#### 1. Environment Drone (`--robot=drone`)

- The drone IS the primary robot
- Spawned by Isaac Lab's environment system
- Uses `QuadcopterEnvCfg` configuration
- Prim path: `/World/envs/env_{i}/Robot`
- ROS2 namespace: `/robot{i}/*`

```bash
./run_sim_drone.sh
# or
python main.py --robot=drone
```

#### 2. World Drone (Companion to Ground Robots)

- Spawned alongside Go2/G1 robots
- Independent from the environment system
- For aerial observation/coordination
- Prim path: `/World/envs/env_0/Drone`
- ROS2 namespace: `/drone/*`

```bash
./run_sim.sh  # Go2 + world drone automatically
# or
python main.py --robot=go2  # World drone spawns automatically
```

### Which to Use?

| Use Case | Command | Drone System |
|----------|---------|--------------|
| Drone-only simulation | `--robot=drone` | Environment drone |
| Ground robot + aerial support | `--robot=go2` | World drone (automatic) |
| Multiple drones | Requires code extension | N/A |

---

## 8. Running the Simulation

### Launch Commands

```bash
# Unitree Go2 (quadruped dog)
./run_sim.sh

# Unitree G1 (humanoid)
./run_sim_g1.sh

# Crazyflie drone only
./run_sim_drone.sh
```

### Command-Line Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `--robot` | `go2` | Robot type: `go2`, `g1`, `drone`, `quadcopter` |
| `--robot_amount` | `2` | Number of robots to spawn |
| `--terrain` | `rough` | Terrain type: `rough`, `flat` |
| `--custom_env` | `office` | Environment: `office`, `warehouse` |
| `--num_envs` | `1` | Number of parallel environments |
| `--calibrate` | `false` | Run drone physics calibration |

### Keyboard Controls

| Key | Ground Robot | Drone |
|-----|--------------|-------|
| `W` | Forward | Forward |
| `S` | Backward | Backward |
| `A` | Strafe left | Strafe left |
| `D` | Strafe right | Strafe right |
| `Q` | Rotate left | Rotate left |
| `E` | Rotate right | Rotate right |
| `T` | - | Altitude up |
| `G` | - | Altitude down |

For second robot (robot1): `I/K/J/L/U/O` and `Y/H` for drone altitude.

---

## 9. Current Gaps and Limitations

### Drone Limitations

| Limitation | Impact | Potential Fix |
|------------|--------|---------------|
| No motor thrust physics | Unrealistic flight dynamics | Use OmniDrones framework |
| No trained RL policy | Manual tuning required | Train with OmniDrones |
| Altitude drift (~2mm/5sec) | Minor position error | Further PID tuning |
| No collision avoidance | Drones can clip through objects | Add collision detection |

### Ground Robot Limitations

| Limitation | Impact | Potential Fix |
|------------|--------|---------------|
| G1 path hardcoded | Must have local USD file | Update `robots/g1/config.py` |
| No high-level navigation | Manual velocity commands only | Integrate Nav2 |
| Single terrain per run | Can't change terrain dynamically | Environment redesign |

### General Limitations

| Limitation | Impact | Potential Fix |
|------------|--------|---------------|
| Custom envs require download | Office/warehouse not included | Add to repo or document |
| VR support commented out | No VR mode | Uncomment and configure |
| Single GPU only | Can't distribute across GPUs | Isaac Lab configuration |

---

## 10. File Reference

### Core Files

| File | Purpose |
|------|---------|
| `main.py` | Entry point, calls `run_sim()` |
| `omniverse_sim.py` | Main simulation loop, robot control, ROS2 integration |
| `custom_rl_env.py` | Environment configurations for each robot type |
| `agent_cfg.py` | RL agent configurations (network architecture, hyperparameters) |
| `drone_controller.py` | PID-based drone flight controller |
| `ros2_bridge.py` | ROS2 publishers and message handling |

### Robot Configurations

| File | Purpose |
|------|---------|
| `robots/g1/config.py` | G1 humanoid articulation configuration |
| `robots/g1/g1.usd` | G1 USD asset file |
| `robots/quadcopter/config.py` | Crazyflie drone articulation configuration |

### Supporting Files

| File | Purpose |
|------|---------|
| `terrain_cfg.py` | Terrain generation configuration |
| `omnigraph.py` | Camera streaming setup |
| `cli_args.py` | Command-line argument parsing |
| `drone_debug_logger.py` | Drone telemetry logging |

### Scripts

| File | Purpose |
|------|---------|
| `run_sim.sh` | Launch Go2 simulation |
| `run_sim_g1.sh` | Launch G1 simulation |
| `run_sim_drone.sh` | Launch drone simulation |
| `calibrate_drone_physics.py` | Drone physics calibration utility |

### Trained Models

| Path | Contents |
|------|----------|
| `logs/rsl_rl/unitree_go2_rough/` | Go2 trained PPO policy |
| `logs/rsl_rl/g1_rough/` | G1 trained PPO policy |

### Documentation Files (Historical)

| File | Status | Notes |
|------|--------|-------|
| `README.md` | Current | Original project overview |
| `DRONE_README.md` | Current | Drone usage guide |
| `SUCCESS_SUMMARY.md` | Current | Working state summary |
| `MIXED_ROBOTS_GUIDE.md` | Current | Ground + aerial scenarios |
| `DIAGNOSTIC_GUIDE.md` | Historical | Early debugging docs |
| `DRONE_SPAWN_ANALYSIS.md` | Historical | Spawn issue investigation |
| `PRODUCTION_BUG_FIX.md` | Reference | GPU physics fix documentation |
| `SENIOR_ENGINEER_REVIEW_FIXES.md` | Reference | Code review fixes |

---

## Appendix: Glossary

| Term | Definition |
|------|------------|
| **USD** | Universal Scene Description - 3D file format for robot/scene assets |
| **Articulation** | A physics object with joints (robot, mechanism) |
| **PPO** | Proximal Policy Optimization - RL training algorithm |
| **PID** | Proportional-Integral-Derivative - classical control algorithm |
| **Isaac Lab** | NVIDIA's robot learning framework (formerly Orbit) |
| **Isaac Sim** | NVIDIA's robotics simulation platform |
| **PhysX** | NVIDIA's physics engine (GPU accelerated) |
| **Prim** | USD primitive - a node in the scene hierarchy |
| **Nucleus** | NVIDIA's cloud storage for Omniverse assets |
| **RL Policy** | A trained neural network that maps observations to actions |
| **Locomotion** | Low-level movement control (how to move legs/motors) |
| **Navigation** | High-level path planning (where to go) |

---

*Document generated: January 2026*
*Last updated to reflect current project state*
