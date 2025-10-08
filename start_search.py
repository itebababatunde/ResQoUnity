"""
Start Search: Isaac Sim simulation with 2 Go2 robots and 1 drone for search and rescue operations.
Uses IsaacLab's proper asset system to spawn real robots.
"""
from __future__ import annotations

import argparse
from omni.isaac.orbit.app import AppLauncher

# Parse arguments
parser = argparse.ArgumentParser(description="Search and Rescue Simulation: 2 Go2 robots + 1 Drone")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# Launch Isaac Sim
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# Now import Isaac Sim modules (must be after AppLauncher)
import omni
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.objects import GroundPlane
from omni.isaac.core.utils.viewports import set_camera_view
import numpy as np
from pxr import UsdLux, Gf, UsdGeom
import omni.usd

# Import IsaacLab robot configurations
import omni.isaac.orbit.sim as sim_utils
from omni.isaac.orbit.assets import Articulation
from omni.isaac.orbit_assets.unitree import UNITREE_GO2_CFG  # The actual Go2 config!

print("="*80)
print("SEARCH AND RESCUE SIMULATION")
print("="*80)
print("[INFO] Initializing simulation...")

# Create world
world = World()

# Add ground plane
print("[INFO] Adding ground plane...")
ground = GroundPlane(prim_path="/World/ground", size=50.0, color=np.array([0.4, 0.4, 0.4]))

# Add lighting
print("[INFO] Adding lights...")
stage = omni.usd.get_context().get_stage()
distant_light = UsdLux.DistantLight.Define(stage, "/World/sunLight")
distant_light.CreateIntensityAttr(1000)
distant_light.CreateAngleAttr(0.5)

# Get assets root
assets_root = get_assets_root_path()
if not assets_root:
    print("[WARN] Could not get assets root")
    assets_root = "http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/2023.1.1"

print(f"[INFO] Assets root: {assets_root}")

# Spawn Go2 robots using IsaacLab's Articulation system
print("[INFO] Spawning Go2 robots using IsaacLab Articulation...")

robot_positions = [
    (-2.0, 0.0, 0.6),  # Robot 1 on the left
    (2.0, 0.0, 0.6),   # Robot 2 on the right
]

robots = []
for i, pos in enumerate(robot_positions):
    try:
        print(f"[INFO] Spawning Go2 robot {i+1} at position {pos}...")
        
        # Create a copy of the Go2 config with unique prim path
        robot_cfg = UNITREE_GO2_CFG.copy()
        robot_cfg.prim_path = f"/World/Go2_{i+1}"
        robot_cfg.init_state.pos = pos
        
        # Spawn the robot using IsaacLab's Articulation
        robot = Articulation(cfg=robot_cfg)
        robots.append(robot)
        
        print(f"[SUCCESS] Go2 robot {i+1} spawned successfully!")
    except Exception as e:
        print(f"[ERROR] Could not spawn Go2 robot {i+1}: {e}")
        import traceback
        traceback.print_exc()

# Spawn drone
print("[INFO] Spawning Crazyflie drone...")
drone_asset_path = f"{assets_root}/Isaac/Robots/Crazyflie/cf2x.usd"
drone_prim_path = "/World/Drone"
drone_position = (0.0, 0.0, 2.5)  # Center, hovering at 2.5m

try:
    add_reference_to_stage(drone_asset_path, drone_prim_path)
    drone_prim = prim_utils.get_prim_at_path(drone_prim_path)
    if drone_prim.IsValid():
        prim_utils.set_prim_attribute_value(drone_prim_path, "xformOp:translate", drone_position)
        # Scale up the drone to make it more visible (Crazyflie is tiny)
        xformable = UsdGeom.Xformable(drone_prim)
        scale_op = xformable.AddScaleOp()
        scale_op.Set(Gf.Vec3d(5.0, 5.0, 5.0))  # Scale 5x for better visibility
        print(f"[SUCCESS] Drone spawned at position {drone_position} (scaled 5x for visibility)")
    else:
        print("[ERROR] Drone prim is not valid")
except Exception as e:
    print(f"[ERROR] Could not load drone: {e}")

# Setup camera view
print("[INFO] Setting up camera view...")
try:
    # Camera positioned to see all robots and drone
    set_camera_view(
        eye=[8.0, 8.0, 5.0],      # Camera position
        target=[0.0, 0.0, 1.0],    # Look at center
        camera_prim_path="/OmniverseKit_Persp"
    )
    print("[SUCCESS] Camera positioned")
except Exception as e:
    print(f"[WARN] Could not set camera view: {e}")

print("\n" + "="*80)
print("SCENE SETUP COMPLETE")
print("="*80)
print("Robots in scene:")
print(f"  - Go2 Robot 1: Left side at {robot_positions[0]}")
print(f"  - Go2 Robot 2: Right side at {robot_positions[1]}")
print("  - Crazyflie Drone: Center hovering at (0.0, 0.0, 2.5)")
print("\nCamera controls:")
print("  - Mouse wheel: Zoom in/out")
print("  - Middle mouse: Pan camera")
print("  - Alt+Left mouse: Orbit view")
print("="*80)

# Reset world to initialize physics
print("\n[INFO] Initializing physics...")
world.reset()

# Initialize robots
print("[INFO] Initializing robots...")
for i, robot in enumerate(robots):
    robot.write_root_pose_to_sim(robot.data.default_root_state[:, :7])
    robot.write_root_velocity_to_sim(robot.data.default_root_state[:, 7:])
    robot.write_joint_state_to_sim(robot.data.default_joint_pos, robot.data.default_joint_vel)
    print(f"[INFO] Robot {i+1} initialized")

# Run simulation
print("[INFO] Starting simulation - Press Ctrl+C to exit")
print("[INFO] Simulation running...")

try:
    step_count = 0
    while simulation_app.is_running():
        # Update robots
        for robot in robots:
            robot.update(dt=world.get_physics_dt())
        
        # Step the world
        world.step(render=True)
        step_count += 1
        
        # Print status every 500 steps
        if step_count % 500 == 0:
            print(f"[INFO] Simulation step {step_count} - All agents active")
            
except KeyboardInterrupt:
    print("\n[INFO] Simulation interrupted by user")

print("[INFO] Shutting down...")
simulation_app.close()
print("[INFO] Done!")
