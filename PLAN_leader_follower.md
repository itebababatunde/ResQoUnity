# Implementation Plan: Leader-Follower Drone-Dog System

## Overview

Implement a two-robot leader-follower system with:
- **Leader**: Quadcopter drone following a lawnmower sweep trajectory
- **Follower**: Unitree Go2 quadruped using APF-based control to follow the drone

Both robots spawn at the same corner of a 10m × 10m square workspace.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                    leader_follower_sim.py                           │
│                      (Main Simulation Loop)                         │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌─────────────────────┐         ┌─────────────────────┐           │
│  │   LawnmowerPlanner  │         │   APFController     │           │
│  │                     │         │                     │           │
│  │  - workspace_size   │         │  - k_att (attract)  │           │
│  │  - sweep_spacing    │         │  - k_rep (repulse)  │           │
│  │  - drone_speed      │         │  - d_safe           │           │
│  │                     │         │  - max_velocity     │           │
│  │  get_position(t) ───┼────────►│  compute_velocity() │           │
│  └─────────────────────┘         └──────────┬──────────┘           │
│           │                                  │                      │
│           ▼                                  ▼                      │
│  ┌─────────────────────┐         ┌─────────────────────┐           │
│  │   DroneController   │         │   Go2 RL Policy     │           │
│  │   (existing PID)    │         │   (velocity_commands│           │
│  │                     │         │    observation)     │           │
│  └─────────────────────┘         └─────────────────────┘           │
│           │                                  │                      │
│           ▼                                  ▼                      │
│  ┌──────────────────────────────────────────────────────┐          │
│  │              Isaac Sim Physics Engine                │          │
│  │         (Quadcopter + Go2 in same scene)            │          │
│  └──────────────────────────────────────────────────────┘          │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Files to Create

### 1. `leader_follower_sim.py` (Main Entry Point)
**Purpose**: Standalone simulation script for the leader-follower demo

**Key Responsibilities**:
- Initialize Isaac Sim with AppLauncher
- Create custom scene with flat terrain (no obstacles)
- Spawn drone and Go2 at corner position (0, 0)
- Run main simulation loop at 60 Hz
- Coordinate between planner and APF controller

### 2. `lawnmower_planner.py` (Drone Trajectory)
**Purpose**: Generate deterministic lawnmower sweep trajectory

**Key Responsibilities**:
- Define workspace boundaries
- Generate waypoints for lawnmower pattern
- Interpolate position at each timestep
- Track coverage progress

**Parameters**:
```python
@dataclass
class LawnmowerConfig:
    workspace_size: float = 10.0      # meters (square)
    sweep_spacing: float = 2.0        # meters between rows
    drone_speed: float = 1.0          # m/s
    drone_altitude: float = 3.0       # meters above ground
    start_corner: tuple = (0.0, 0.0)  # spawn position
```

**Algorithm**:
```
Row 0: (0,0) → (10,0)
Row 1: (10,2) → (0,2)
Row 2: (0,4) → (10,4)
...continue until full coverage
```

### 3. `apf_controller.py` (Follower Control)
**Purpose**: Compute APF-based velocity commands for the Go2

**Key Responsibilities**:
- Compute attractive force toward drone
- Compute repulsive force when too close
- Combine forces into velocity command
- Clamp to motion limits

**Parameters**:
```python
@dataclass
class APFConfig:
    k_att: float = 1.0          # Attractive gain
    k_rep: float = 2.0          # Repulsive gain
    d_safe: float = 2.0         # Minimum separation (meters)
    d_influence: float = 5.0    # Repulsion influence range
    max_velocity: float = 1.5   # m/s (Go2 max walking speed)
    velocity_deadband: float = 0.1  # Ignore tiny velocities
```

**Control Law Implementation**:
```python
def compute_velocity(self, drone_pos: np.ndarray, dog_pos: np.ndarray) -> np.ndarray:
    """
    Compute APF-based velocity command for the dog.

    Args:
        drone_pos: [x, y] position of drone (leader)
        dog_pos: [x, y] position of dog (follower)

    Returns:
        velocity: [vx, vy] velocity command for dog
    """
    # Vector from dog to drone
    to_drone = drone_pos - dog_pos
    distance = np.linalg.norm(to_drone)

    if distance < 0.001:
        return np.array([0.0, 0.0])

    direction = to_drone / distance

    # Attractive force (always active)
    F_att = self.k_att * to_drone

    # Repulsive force (active when d < d_safe)
    F_rep = np.array([0.0, 0.0])
    if distance < self.d_safe:
        # Repulsion formula: k_rep * (1/d - 1/d_safe) * (1/d^2) * direction_away
        repulsion_magnitude = self.k_rep * (1.0/distance - 1.0/self.d_safe) * (1.0 / distance**2)
        F_rep = -repulsion_magnitude * direction  # Away from drone

    # Total force
    F_total = F_att + F_rep

    # Convert force to velocity (simple proportional)
    velocity = F_total

    # Clamp to max velocity
    speed = np.linalg.norm(velocity)
    if speed > self.max_velocity:
        velocity = velocity / speed * self.max_velocity

    # Apply deadband
    if speed < self.velocity_deadband:
        velocity = np.array([0.0, 0.0])

    return velocity
```

### 4. `leader_follower_env.py` (Environment Config)
**Purpose**: Custom Isaac Lab environment configuration

**Key Responsibilities**:
- Define scene with flat terrain
- Configure both robot articulations
- Set up observation/action spaces
- Handle multi-robot spawning at same position

**Scene Setup**:
```python
@configclass
class LeaderFollowerSceneCfg(InteractiveSceneCfg):
    # Flat ground plane
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(size=(20.0, 20.0)),
    )

    # Drone (leader)
    drone: ArticulationCfg = QUADCOPTER_CFG.replace(
        prim_path="/World/drone",
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 3.0),  # Start at corner, 3m altitude
        ),
    )

    # Go2 (follower)
    dog: ArticulationCfg = UNITREE_GO2_CFG.replace(
        prim_path="/World/dog",
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.4),  # Start at same corner, on ground
        ),
    )
```

---

## Integration with Existing Code

### Reused Components

| Component | Source File | Usage |
|-----------|-------------|-------|
| `QUADCOPTER_CFG` | `robots/quadcopter/config.py` | Drone articulation config |
| `UNITREE_GO2_CFG` | `omni.isaac.orbit_assets` | Go2 articulation config |
| `DroneController` | `drone_controller.py` | PID position control for drone |
| `calculate_drone_forces` | `omniverse_sim.py` | Force-based velocity control |
| Go2 RL Policy | Trained model | Locomotion from velocity commands |

### Go2 Velocity Command Interface

The existing Go2 RL policy accepts velocity commands via the `velocity_commands` observation:
```python
# From custom_rl_env.py observation manager
"velocity_commands": ObsTerm(func=mdp.generated_commands, params={"command_name": "base_velocity"})
```

The APF controller output maps to:
```python
# APF output [vx, vy] maps to base_command
custom_rl_env.base_command[dog_id] = [apf_vx, apf_vy, 0.0]  # [lin_x, lin_y, ang_z]
```

---

## Simulation Loop Pseudocode

```python
def run_leader_follower():
    # Initialization
    app_launcher = AppLauncher(args)
    env = create_environment()

    drone = env.scene["drone"]
    dog = env.scene["dog"]

    planner = LawnmowerPlanner(LawnmowerConfig())
    apf = APFController(APFConfig())
    drone_ctrl = DroneController(...)

    # Load Go2 RL policy
    go2_policy = load_policy("go2_flat_policy.pt")

    sim_time = 0.0
    dt = 1.0 / 60.0  # 60 Hz

    while simulation_app.is_running():
        # 1. Get drone target from planner
        drone_target = planner.get_position(sim_time)

        # 2. Update drone controller to track target
        drone_ctrl.set_target_position(drone_target)
        drone_velocity = drone_ctrl.update(dt, drone.position, drone.velocity)

        # 3. Get current positions (2D projection)
        drone_pos_2d = drone.position[:2]
        dog_pos_2d = dog.position[:2]

        # 4. Compute APF velocity for dog
        dog_velocity_cmd = apf.compute_velocity(drone_pos_2d, dog_pos_2d)

        # 5. Set dog velocity command for RL policy
        base_command["dog"] = [dog_velocity_cmd[0], dog_velocity_cmd[1], 0.0]

        # 6. Get observations and run policies
        obs = env.get_observations()

        # Drone: apply force-based control
        apply_drone_forces(drone, drone_velocity)

        # Dog: run RL policy with APF velocity command
        dog_actions = go2_policy(obs["dog"])

        # 7. Step physics
        env.step({"drone": zero_actions, "dog": dog_actions})

        # 8. Log/visualize
        log_state(sim_time, drone.position, dog.position)

        # 9. Check completion
        if planner.is_complete():
            print("Coverage complete!")
            break

        sim_time += dt
```

---

## Parameters Summary

### System Parameters
| Parameter | Value | Description |
|-----------|-------|-------------|
| `dt` | 0.0167s | Timestep (60 Hz) |
| `workspace_size` | 10.0m | Square side length |
| `spawn_corner` | (0, 0) | Both robots start here |

### Drone (Leader) Parameters
| Parameter | Value | Description |
|-----------|-------|-------------|
| `altitude` | 3.0m | Fixed flight altitude |
| `speed` | 1.0 m/s | Traversal velocity |
| `sweep_spacing` | 2.0m | Distance between rows |

### Dog (Follower) APF Parameters
| Parameter | Value | Description |
|-----------|-------|-------------|
| `k_att` | 1.0 | Attractive force gain |
| `k_rep` | 2.0 | Repulsive force gain |
| `d_safe` | 2.0m | Minimum separation distance |
| `max_velocity` | 1.5 m/s | Go2 max walking speed |

---

## Implementation Order

### Phase 1: Core Components
1. [ ] Create `lawnmower_planner.py` with trajectory generation
2. [ ] Create `apf_controller.py` with force computation
3. [ ] Create `leader_follower_env.py` with scene configuration

### Phase 2: Simulation Integration
4. [ ] Create `leader_follower_sim.py` main script
5. [ ] Integrate drone PID control from existing code
6. [ ] Integrate Go2 RL policy with APF velocity commands
7. [ ] Set up flat terrain scene with both robots

### Phase 3: Testing & Tuning
8. [ ] Test drone lawnmower trajectory alone
9. [ ] Test APF controller with stationary drone
10. [ ] Full leader-follower simulation
11. [ ] Tune APF gains for smooth following

### Phase 4: Visualization & Logging
12. [ ] Add trajectory visualization
13. [ ] Log positions and separation distance
14. [ ] Add coverage progress indicator

---

## Run Command

```bash
python leader_follower_sim.py --headless  # For faster simulation
python leader_follower_sim.py             # With visualization
```

---

## Success Criteria

1. **Drone completes full lawnmower sweep** of 10m × 10m workspace
2. **Dog maintains following behavior** throughout traversal
3. **Minimum separation never violated** (d ≥ d_safe always)
4. **Smooth motion** without oscillations or jerky behavior
5. **Simulation runs at real-time** or faster

---

## Risks and Mitigations

| Risk | Mitigation |
|------|------------|
| Go2 RL policy may not respond well to rapid APF velocity changes | Add velocity smoothing/filtering to APF output |
| Drone position control may have lag | Tune PID gains; consider feedforward |
| APF oscillation near d_safe boundary | Add damping term to APF; tune gains |
| Go2 may not reach max velocity 1.5 m/s | Reduce drone speed or increase following distance |
| Scene with two articulations may have performance issues | Use single environment, disable unnecessary sensors |

---

## Questions Resolved

- ✅ Dog model: Unitree Go2 with RL policy
- ✅ Integration: New standalone simulation script
- ✅ Workspace: 10m × 10m
- ✅ Traversal: Lawnmower sweep pattern
