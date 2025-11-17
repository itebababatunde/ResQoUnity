# Deep Research Request: Isaac Sim GPU PhysX Viewport Update Issue

## Problem Summary

We are developing a drone control system in NVIDIA Isaac Sim 2023.1.1 using Isaac Lab (Orbit). The drone's **physics is working perfectly** (position changes are logged correctly), but the **viewport rendering stays frozen** - the drone appears motionless despite actually moving in the physics simulation.

## Technical Environment

- **Platform**: NVIDIA Isaac Sim 2023.1.1
- **Framework**: Isaac Lab (omni.isaac.orbit)
- **Physics**: GPU PhysX (`Physics GPU simulation: True`)
- **Robot Setup**: 
  - Primary robot: Unitree Go2 (managed by environment)
  - Secondary robot: Quadcopter drone (world-level, at `/World/Drone`)
- **Control Method**: Hybrid physics-velocity control (F=ma → acceleration → velocity integration)

## What We've Discovered

### Physics Works Correctly ✅
```
Logs show position changing:
FRAME 4:  Position = 2.497m
FRAME 21: Position = 2.447m (-5.0cm)
FRAME 40: Position = 2.424m (-2.3cm)
```

Physics velocities are applied, internal position buffers update, and odometry published via ROS2 reflects correct movement.

### Viewport Rendering Frozen ❌
Despite position changes in logs, the drone appears completely stationary in the Isaac Sim viewport. User can see the drone hovering but no visual movement occurs.

## What We've Tried

### Attempt 1: Manual Position Updates via `set_world_poses()`
```python
new_pos = current_pos + velocity * dt
world_drone_view.set_world_poses(new_pos, orientations)
# Result: Succeeds without error, but viewport doesn't update
```

### Attempt 2: Force USD Stage Sync via `write_root_pose_to_sim()`
```python
world_drone_view.write_root_pose_to_sim(new_pos, orientations)
# Result: Method doesn't exist in ArticulationView
```

### Attempt 3: Remove Joint Velocity Targets
Initially suspected `set_joint_velocity_targets()` was locking the drone, but removing it didn't fix the viewport freeze (though it did fix an earlier physics issue).

### Attempt 4: Dynamic Camera Tracking
Attempted to make camera follow drone to verify movement, but drone confirmed to be in fixed position visually even with manual camera navigation.

## Root Cause Hypothesis

**GPU PhysX owns all articulation transforms and only updates viewport when physics steps that specific object.**

Key observations:
1. `set_world_poses()` updates internal ArticulationView buffers (we can read changed values back)
2. Viewport rendering only syncs when physics engine **steps** that articulation
3. World drone is spawned at `/World/Drone` (outside environment scene)
4. `env.step(actions)` may only step objects within `/World/envs/env_0/` 
5. Therefore: Physics calculates correct positions internally, but viewport never receives the update

## Current Architecture

```python
# Drone spawning (world-level, outside environment)
prim_utils.create_prim(
    prim_path="/World/Drone",  # NOT inside /World/envs/env_0/
    usd_path=drone_usd_path,
    translation=(0.0, 0.0, 2.5)
)

# Drone control happens AFTER env.step()
obs, _, _, _ = env.step(actions)  # Steps environment robots only?

# Read drone state
positions, orientations = world_drone_view.get_world_poses()

# Calculate and apply velocities
new_velocities = calculate_from_pid(...)
world_drone_view.set_velocities(new_velocities)

# Next frame: env.step() may not step /World/Drone physics!
```

## Research Questions

### 1. Core Technical Questions
- **Q1**: In Isaac Sim GPU PhysX mode, how do you force viewport rendering to sync with manually-updated ArticulationView positions?
- **Q2**: Does `env.step()` in Isaac Lab only step articulations within the environment scene (`/World/envs/env_X/`)?
- **Q3**: Is there a way to manually step physics for world-level articulations outside the environment?
- **Q4**: What's the correct API to update visual transforms for GPU PhysX articulations?

### 2. Architecture Questions
- **Q5**: Should world-level robots be spawned inside or outside the environment scene for proper physics stepping?
- **Q6**: Is there a `simulation_app.update()` or similar method to manually step world-level physics?
- **Q7**: Can ArticulationView objects participate in physics simulation without being part of an Isaac Lab environment?

### 3. Similar Implementation Research
- **Q8**: Are there examples of multi-robot setups in Isaac Sim where one robot is environment-managed and another is world-level?
- **Q9**: How do other Isaac Sim projects handle drone control with GPU PhysX?
- **Q10**: Has anyone solved viewport sync issues with manually-controlled articulations in GPU PhysX mode?

## Specific Search Queries Needed

Please search for:
1. **Isaac Sim GPU PhysX viewport update** issues and solutions
2. **ArticulationView rendering sync** with physics in GPU mode
3. **World-level vs environment-level robots** in Isaac Lab
4. **Manual physics stepping** for non-environment objects in Isaac Sim
5. **Multi-robot Isaac Sim examples** with mixed control methods
6. **OmniDrones** implementation (they do drone control in Isaac Sim - how do they handle rendering?)
7. **Isaac Lab environment scene** structure and what `env.step()` actually steps
8. Any GitHub repositories with **working drone control in Isaac Sim 2023+**

## Expected Outcome

We need to find:
1. **The correct API/method** to ensure viewport updates when physics state changes
2. **Example code** from similar implementations that work
3. **Confirmation** of whether our architecture (world-level drone + environment robot) is fundamentally incompatible with GPU PhysX
4. **Alternative approaches** if current method is impossible

## Drone Asset Details

**Using NVIDIA Official Crazyflie Asset** (NOT custom):
```
USD Path: http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/2023.1.1/Isaac/Robots/Crazyflie/cf2x.usd
Scale: 5x (for visibility - Crazyflie is tiny)
```

**Physics Properties Configured**:
- ✅ Articulation root with 4 revolute joints (m1-m4 rotors)
- ✅ Rigid body properties: gravity enabled, linear_damping=0.01, angular_damping=0.05
- ✅ Articulation properties: self-collisions disabled, 4 solver iterations
- ✅ Implicit actuators on all 4 rotor joints (stiffness=0, damping=0.01)

**The USD asset is complete and verified working** - it's NVIDIA's official asset used in their examples. The issue is NOT with the asset itself.

## Code References

- Main simulation file: `omniverse_sim.py` (1472 lines)
- Drone controller: `drone_controller.py` (PID-based control with multiple flight modes)
- Drone config: `robots/quadcopter/config.py` (uses official Crazyflie USD)
- Spawning logic: Lines 730-810 in `omniverse_sim.py`
- Control logic: Lines 1120-1330 in `omniverse_sim.py`
- Physics: Hybrid force-velocity approach (F=ma → v_new = v_current + a*dt)

## Additional Context

- We've spent multiple debugging sessions eliminating false leads (double-damping, joint locks, camera issues)
- Physics logs prove the simulation is calculating correctly
- The ONLY issue is viewport rendering not syncing with internal physics state
- This suggests a fundamental misunderstanding of GPU PhysX rendering pipeline

---

**Please conduct deep research across:**

## Research Priority Order

### TIER 1: Official NVIDIA Sources (Search First)
1. **Isaac Sim 2023.1.1 Documentation**
   - GPU PhysX specific documentation
   - ArticulationView API reference
   - Multi-robot simulation guides
   - Scene hierarchy and physics stepping

2. **Isaac Lab (Orbit) Source Code**
   - How `env.step()` works internally
   - What objects get physics stepped
   - Multi-robot examples in codebase
   - World-level object handling

3. **NVIDIA Official Examples**
   - Isaac Sim example scripts
   - Isaac Lab example environments
   - Multi-agent scenarios
   - Drone/aerial robot examples

### TIER 2: Community & Implementation Research
4. **GitHub Repositories**
   - OmniDrones project (drone control in Isaac Sim)
   - Isaac Lab forks and extensions
   - Multi-robot Isaac Sim projects
   - Aerial robotics implementations

5. **NVIDIA Forums & Community**
   - Isaac Sim forums (forums.developer.nvidia.com)
   - Omniverse forums
   - GitHub Issues in Isaac Lab repo
   - Stack Overflow (isaac-sim, omniverse tags)

6. **Academic Papers & Technical Blogs**
   - Papers using Isaac Sim for multi-robot
   - Technical blog posts on Isaac Sim architecture
   - Drone simulation case studies

## Workaround Acceptance

**YES - Include ALL potential workarounds**, including:

✅ **Scene hierarchy restructuring** (e.g., moving drone to `/World/envs/env_0/Drone`)
✅ **Manual physics stepping methods** (if any exist)
✅ **Alternative control architectures** (e.g., drone as primary robot)
✅ **Hybrid approaches** (e.g., combining environment and world-level objects)
✅ **API alternatives** (e.g., using different Isaac Sim APIs)
✅ **Rendering pipeline hacks** (if documented and safe)

**We want to see ALL possible solutions**, even if they require significant refactoring. The goal is to understand:
1. What's possible within current architecture
2. What requires architectural changes
3. What's fundamentally impossible with GPU PhysX

## Specific Questions to Answer

From research, please determine:

1. ✅ **Can world-level articulations be stepped independently?** (with code examples)
2. ✅ **Does spawning in environment scene solve the issue?** (with evidence)
3. ✅ **How does env.step() decide what to step?** (from source code)
4. ✅ **Are there GPU PhysX limitations on manual transform updates?** (from docs)
5. ✅ **How do other projects handle this?** (with links to working code)
6. ✅ **What's the "correct" architecture for multi-robot setups?** (best practices)

---

**Goal**: Find the definitive solution or confirm if this architecture is impossible, requiring a redesign. Prioritize working solutions over theoretical ones.

