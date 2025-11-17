#!/usr/bin/env python3
"""
ABSOLUTE SIMPLEST DRONE TEST
No Go2, no world-level spawning, just pure drone as environment robot.
This will PROVE if drone control works or is fundamentally broken.
"""

import torch
import gymnasium as gym

# Isaac Lab imports
from omni.isaac.lab_tasks.utils import parse_env_cfg
from omni.isaac.lab.envs import ManagerBasedRLEnv

# Custom environment
import custom_rl_env
from custom_rl_env import QuadcopterEnvCfg

# Drone controller
from drone_controller import DroneController

def main():
    print("=" * 80)
    print("SIMPLEST DRONE TEST - No Go2, Just Drone")
    print("=" * 80)
    
    # Create environment with DRONE only
    print("\n[1/4] Creating drone environment...")
    env_cfg = QuadcopterEnvCfg()
    env_cfg.scene.num_envs = 1  # Just 1 drone
    
    env = gym.make("Isaac-Velocity-Rough-Unitree-Go2-v0", cfg=env_cfg)
    print("✓ Environment created")
    
    # Reset
    print("\n[2/4] Resetting environment...")
    obs, _ = env.reset()
    print("✓ Environment reset")
    
    # Initialize drone controller
    print("\n[3/4] Initializing drone controller...")
    controller = DroneController(
        robot_id="0",
        kp_pos=0.3, ki_pos=0.0, kd_pos=0.2,
        kp_alt=0.8, ki_alt=0.0, kd_alt=0.4
    )
    controller.arm()
    print("✓ Drone armed")
    
    # Get robot from scene
    robot = env.unwrapped.scene["robot"]
    
    print("\n[4/4] Running simulation loop...")
    print("Commands:")
    print("  - Drone will try to hold position at current height")
    print("  - After 100 steps, will command takeoff to 3m")
    print("  - Watch Isaac Sim viewport!")
    print("\nStarting in 3 seconds...")
    import time
    time.sleep(3)
    
    for step in range(300):
        # Get current state
        positions, orientations = robot.data.root_pos_w, robot.data.root_quat_w
        velocities = robot.data.root_lin_vel_w
        
        current_pos = positions[0].cpu().numpy()
        current_vel = velocities[0].cpu().numpy()
        
        # Update controller
        controller.current_position = current_pos
        controller.current_velocity = current_vel
        controller.update(0.02, current_pos, current_vel)  # 20ms timestep
        
        # At step 100, command takeoff
        if step == 100:
            print(f"\n[TAKEOFF] Commanding drone to Z=3.0m")
            controller.set_target_position([current_pos[0], current_pos[1], 3.0])
        
        # Get motor commands
        motor_cmds = controller.compute_motor_command()
        
        # Convert to actions
        actions = torch.tensor([[motor_cmds[0], motor_cmds[1], motor_cmds[2], motor_cmds[3]]], 
                                device=robot.device, dtype=torch.float32)
        
        # Step environment
        obs, _, _, _ = env.step(actions)
        
        # Log every 50 steps
        if step % 50 == 0:
            print(f"Step {step:3d}: Pos=({current_pos[0]:.2f}, {current_pos[1]:.2f}, {current_pos[2]:.2f}) "
                  f"Vel=({current_vel[0]:.2f}, {current_vel[1]:.2f}, {current_vel[2]:.2f})")
    
    print("\n" + "=" * 80)
    print("TEST COMPLETE")
    print("=" * 80)
    print("\nDid you see the drone move in Isaac Sim viewport?")
    print("  YES → Drone control works! Issue is multi-robot setup")
    print("  NO  → Drone control is fundamentally broken")
    
    env.close()

if __name__ == "__main__":
    main()

