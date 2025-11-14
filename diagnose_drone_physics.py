#!/usr/bin/env python3
"""
Diagnostic script to determine WHY the drone isn't moving.
Run this to get actionable data.
"""

print("="*80)
print("DRONE PHYSICS DIAGNOSTIC")
print("="*80)

print("\n1. Checking simulation logs for control loop activity...")
print("   → Run simulation and look for '[DIAG]' messages")
print("   → If no '[DIAG]' messages: control loop isn't running")
print("   → If '[DIAG]' messages present: control loop is running")

print("\n2. Checking if ArticulationView methods are being called...")
print("   → Look for 'set_world_poses' or 'set_velocities' in output")
print("   → If present: methods are being called")
print("   → If absent: methods aren't being reached")

print("\n3. Check if position changes between frames:")
print("   → Look for '[DIAG] Before set' and '[DIAG] After set'")
print("   → If 'Position changed: True': ArticulationView works!")
print("   → If 'Position changed: False': ArticulationView is broken/ignored")

print("\n4. Check ROS2 odometry for ANY position changes:")
print("   → Terminal 1: Start simulation")
print("   → Terminal 2: ros2 topic echo /drone/odom | grep -A3 position")
print("   → Send takeoff command")
print("   → Watch if Z coordinate ever changes")

print("\n5. Check if PhysX is simulating the standalone drone:")
print("   → Look for PhysX errors mentioning '/World/Drone'")
print("   → Error about 'eENABLE_DIRECT_GPU_API' means GPU mode active")
print("   → Standalone prims might not be simulated by env.step()")

print("\n" + "="*80)
print("DIAGNOSIS DECISION TREE:")
print("="*80)

print("""
┌─ Control loop not running?
│  └─> Controller not initialized OR exception in try block
│
├─ Control loop running but position never changes?
│  ├─ ArticulationView methods fail silently
│  │  └─> GPU PhysX restriction
│  ├─ ArticulationView controls wrong prim
│  │  └─> Check prim_paths_expr
│  └─ Standalone drone not being stepped by physics
│     └─> env.step() only steps environment robots
│
└─ Position changes internally but ROS2 shows frozen?
   └─> Publishing stale cache (already fixed)
""")

print("\n" + "="*80)
print("ACTION ITEMS:")
print("="*80)

print("""
1. Add diagnostic prints to omniverse_sim.py:
   - Frame counter to prove control loop runs
   - Before/after position to prove set_world_poses works
   
2. Run simulation and collect output

3. Based on output, we'll know:
   - Is code even running?
   - Do ArticulationView setters work?
   - Is physics stepping the drone?

4. Then we can fix the ROOT cause, not guess.
""")

print("\nReady to add diagnostics? (y/n): ", end='')
response = input().strip().lower()

if response == 'y':
    print("\n✅ Adding diagnostic code to omniverse_sim.py...")
    
    # Read the file
    with open('omniverse_sim.py', 'r') as f:
        lines = f.readlines()
    
    # Find the line with "try:" around line 1254
    insert_after_line = None
    for i, line in enumerate(lines):
        if 'Convert forces to acceleration: a = F/m' in line:
            # Found the start of the physics block
            insert_after_line = i + 1
            break
    
    if insert_after_line:
        # Insert diagnostic code
        diagnostic_code = """                                    # ============ DIAGNOSTICS START ============
                                    if not hasattr(custom_rl_env, 'world_drone_frame_count'):
                                        custom_rl_env.world_drone_frame_count = 0
                                    custom_rl_env.world_drone_frame_count += 1
                                    
                                    if custom_rl_env.world_drone_frame_count % 60 == 0:  # Every second
                                        print(f"\\n[DIAG] ===== FRAME {custom_rl_env.world_drone_frame_count} =====")
                                        print(f"[DIAG] Control loop is RUNNING")
                                        print(f"[DIAG] Current position: {current_pos}")
                                        print(f"[DIAG] Target position: {controller.target_position}")
                                        print(f"[DIAG] Position error: {controller.target_position - current_pos}")
                                        print(f"[DIAG] Desired velocity: ({desired_vx:.3f}, {desired_vy:.3f}, {desired_vz:.3f})")
                                        print(f"[DIAG] Calculated forces: {forces}")
                                        print(f"[DIAG] Mode: {controller.mode.value}, Armed: {controller.armed}")
                                    # ============ DIAGNOSTICS END ============
                                    
"""
        lines.insert(insert_after_line, diagnostic_code)
        
        # Write back
        with open('omniverse_sim.py', 'w') as f:
            f.writelines(lines)
        
        print("✅ Diagnostics added!")
        print("\nNow run your simulation and paste the [DIAG] output here.")
        print("This will tell us EXACTLY what's happening.")
    else:
        print("❌ Couldn't find insertion point. Add manually after line with 'Convert forces to acceleration'")
else:
    print("\n📋 Manual steps:")
    print("1. Open omniverse_sim.py")
    print("2. Find line ~1256: 'acceleration = forces / drone_mass'")
    print("3. Add diagnostic prints there")
    print("4. Run simulation")
    print("5. Paste [DIAG] output")

