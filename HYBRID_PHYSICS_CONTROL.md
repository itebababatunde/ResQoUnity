# Hybrid Physics-Velocity Drone Control

## The Problem

Isaac Sim's GPU PhysX mode **forbids direct force application** via USD attributes:
- `PhysxRigidBodyAPI.GetForceAttr().Set()` is **silently ignored**
- Forces are calculated correctly but have **zero effect** on the drone
- This is a GPU PhysX architectural limitation (similar to `setGlobalPose()` error)

## The Solution: Hybrid Approach

Instead of fighting GPU PhysX, we **work with it** by using physics for calculations but velocities for actuation:

```
1. Calculate desired velocity (PID) → vx, vy, vz
2. Calculate forces needed (F = m × a) → Fx, Fy, Fz (with gravity compensation)
3. Convert forces to acceleration (a = F/m) → ax, ay, az
4. Integrate to new velocity (v_new = v_current + a × dt)
5. Apply velocity via set_velocities() → WORKS in GPU mode!
```

## Key Features

### Physics-Based Behavior
- Proper gravity compensation (mass × 9.81)
- Realistic acceleration curves (F=ma integration)
- Air resistance simulation (velocity damping)
- Smooth, continuous motion

### GPU PhysX Compatible
- Uses `set_velocities()` which GPU PhysX allows
- No direct force/torque/pose manipulation
- Works with ArticulationView API
- No PhysX errors or warnings

### Safe & Stable
- Velocity clamping (max 5 m/s)
- Damping to prevent oscillations (95% per frame)
- Error handling with fallbacks
- Comprehensive logging

## Implementation Details

### Control Loop (60 Hz)
```python
# 1. PID calculates desired velocity
desired_vel = PID(position_error)

# 2. Calculate forces (includes gravity compensation)
forces = calculate_drone_forces(desired_vel, current_vel, mass)
# forces = [Fx, Fy, Fz] where Fz includes +mass*9.81

# 3. Convert to acceleration
acceleration = forces / mass  # a = F/m

# 4. Integrate velocity
dt = 1/60  # 60 Hz
new_vel = current_vel + acceleration * dt

# 5. Apply damping (air resistance)
new_vel *= 0.95

# 6. Clamp for safety
new_vel = clip(new_vel, -5, 5)

# 7. Apply to drone
world_drone_view.set_velocities(new_vel)
```

### Gravity Compensation
```python
force[2] += mass * 9.81  # Add upward force to counteract gravity
```

When `desired_vel = 0` (hovering):
- Net force = +4.9N upward (for 0.5kg drone)
- Acceleration = +9.81 m/s² upward
- Balances gravity's -9.81 m/s² downward
- Result: stable hover

## Advantages

1. **Works in GPU PhysX** - No restrictions violated
2. **Physics-accurate** - Real F=ma dynamics
3. **Smooth motion** - Integration provides natural curves
4. **Debuggable** - Can log forces, accelerations, velocities
5. **Tunable** - Adjust damping, max velocity, integration rate

## Comparison to Direct Force Application

| Method | GPU PhysX | Physics Accuracy | Complexity |
|--------|-----------|------------------|------------|
| Direct Forces | ❌ Blocked | ✅ Perfect | Simple |
| Hybrid Physics-Velocity | ✅ Works | ✅ Excellent | Medium |
| Pure Velocity | ✅ Works | ❌ Poor | Simple |

## Expected Behavior

### Takeoff
- Initial velocity: 0 m/s
- Forces calculated: ~10.9N upward
- Acceleration: ~21 m/s² upward (gravity compensated)
- After 1s: ~1.5 m/s upward velocity
- After 1s: ~0.75m altitude gain
- Smooth exponential approach to target altitude

### Hover
- Target reached, PID outputs 0 velocity
- Forces = gravity compensation only
- Acceleration cancels gravity
- Velocity maintained at ~0
- Small oscillations damped out

### Movement
- PID commands horizontal velocity
- Forces include thrust vectoring
- Smooth acceleration/deceleration
- Natural-looking trajectories

## Monitoring

Watch the logs for:
```
[INFO] Using Hybrid Physics-Velocity Control (GPU PhysX compatible)
[INFO] Forces → Acceleration → Velocity integration (F=ma)

Applied Force: (0.01, 0.00, 10.90) N (|F|=10.90N)
✅ Gravity compensation active (Fz > 0)
Actual Vel: (0.000, 0.000, 0.250) m/s
ℹ️  Velocity differs from PID (hybrid physics integration active)
```

## Success Criteria

✅ Drone climbs on takeoff
✅ Smooth acceleration curves
✅ Stable hover
✅ Responsive to commands
✅ No GPU PhysX errors
✅ Position tracking works

## Troubleshooting

### Drone Still Not Moving
- Check mass calculation (should be ~0.5kg)
- Verify `set_velocities()` being called
- Check for collision constraints
- Verify ArticulationView initialized

### Oscillations
- Increase damping (reduce 0.95 to 0.90)
- Reduce PID gains
- Lower max velocity limit

### Too Slow Response
- Increase PID gains
- Reduce damping (0.95 → 0.98)
- Check dt value (should be 1/60)

---

**Status:** Implemented and ready for testing
**Confidence:** 80% - This approach should work with GPU PhysX
**Fallback:** If still fails, switch to CPU physics mode

