"""
Leader-Follower Simulation with RL Locomotion Policy

This version uses the trained Go2 RL policy for locomotion, with APF providing
velocity commands that feed into the policy's observation space.

Usage:
    python leader_follower_rl_sim.py              # With visualization
    python leader_follower_rl_sim.py --headless   # Headless mode
"""

from __future__ import annotations

import argparse
from omni.isaac.orbit.app import AppLauncher

# Command line arguments
parser = argparse.ArgumentParser(description="Leader-Follower with RL Locomotion")
parser.add_argument("--cpu", action="store_true", default=False, help="Use CPU pipeline")
parser.add_argument("--disable_fabric", action="store_true", default=False, help="Disable fabric")
parser.add_argument("--num_envs", type=int, default=1, help="Number of parallel environments")
parser.add_argument("--seed", type=int, default=None, help="Random seed")
parser.add_argument("--task", type=str, default="Isaac-Velocity-Rough-Unitree-Go2-v0", help="RL task")

# Leader-follower parameters
parser.add_argument("--workspace_size", type=float, default=10.0, help="Workspace side length (meters)")
parser.add_argument("--sweep_spacing", type=float, default=2.0, help="Lawnmower row spacing (meters)")
parser.add_argument("--drone_speed", type=float, default=0.8, help="Drone traversal speed (m/s)")
parser.add_argument("--drone_altitude", type=float, default=3.0, help="Drone flight altitude (meters)")
parser.add_argument("--d_safe", type=float, default=2.0, help="Minimum separation distance (meters)")
parser.add_argument("--k_att", type=float, default=0.8, help="APF attractive gain")
parser.add_argument("--k_rep", type=float, default=2.0, help="APF repulsive gain")
parser.add_argument("--max_dog_velocity", type=float, default=1.2, help="Max dog velocity (m/s)")
parser.add_argument("--log_interval", type=float, default=1.0, help="Logging interval (seconds)")

# AppLauncher arguments
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# Launch Isaac Sim
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# Import remaining modules after app launch
import numpy as np
import torch
import gymnasium as gym
from datetime import datetime
import os

import omni.isaac.orbit.sim as sim_utils
from omni.isaac.orbit_tasks.utils import get_checkpoint_path
from omni.isaac.orbit_tasks.utils.wrappers.rsl_rl import RslRlVecEnvWrapper
from rsl_rl.runners import OnPolicyRunner

# Local imports
from lawnmower_planner import LawnmowerPlanner, LawnmowerConfig
from apf_controller import APFController, APFConfig
from drone_controller import DroneController, DroneState
import agent_cfg as agent_cfg_module
import custom_rl_env


def calculate_drone_forces(desired_velocity, current_velocity, mass, dt=0.016, gravity=9.81):
    """Calculate forces for drone velocity control."""
    velocity_error = desired_velocity - current_velocity
    kp_accel = 15.0
    desired_accel = np.clip(kp_accel * velocity_error, -15.0, 15.0)
    force = mass * desired_accel
    force[2] += mass * gravity
    return force


class LeaderFollowerSystem:
    """
    Manages the leader-follower coordination between drone and Go2.

    The drone follows a pre-planned lawnmower trajectory.
    The Go2 uses APF-based velocity commands fed into its RL locomotion policy.
    """

    def __init__(self, args):
        self.args = args

        # Initialize planner
        self.planner = LawnmowerPlanner(LawnmowerConfig(
            workspace_size=args.workspace_size,
            sweep_spacing=args.sweep_spacing,
            drone_speed=args.drone_speed,
            drone_altitude=args.drone_altitude,
            start_corner=(0.0, 0.0),
        ))

        # Initialize APF controller
        self.apf = APFController(APFConfig(
            k_att=args.k_att,
            k_rep=args.k_rep,
            d_safe=args.d_safe,
            max_velocity=args.max_dog_velocity,
        ))

        # Drone controller for position tracking
        self.drone_ctrl = DroneController(
            robot_id="leader",
            kp_pos=0.5,
            ki_pos=0.0,
            kd_pos=0.3,
            max_vel=args.drone_speed * 1.5,
        )
        self.drone_ctrl.armed = True
        self.drone_ctrl.set_mode(DroneState.POSITION)

        # State tracking
        self.sim_time = 0.0
        self.drone_mass = 1.5  # kg

        # Logging
        self.log_data = {
            'time': [],
            'drone_pos': [],
            'dog_pos': [],
            'separation': [],
            'dog_cmd': [],
            'progress': [],
        }

        print(f"\nLeader-Follower System Initialized")
        print(f"  Planner: {self.planner}")
        print(f"  APF: {self.apf}")
        print(f"  Expected duration: {self.planner.total_time:.1f}s")

    def get_drone_target(self) -> np.ndarray:
        """Get current drone target position from planner."""
        return self.planner.get_position(self.sim_time)

    def compute_drone_velocity(self, current_pos: np.ndarray, current_vel: np.ndarray, dt: float) -> np.ndarray:
        """Compute drone velocity command to track trajectory."""
        target = self.get_drone_target()
        error = target - current_pos
        desired_vel = np.clip(error * 2.0, -self.args.drone_speed, self.args.drone_speed)

        forces = calculate_drone_forces(desired_vel, current_vel, self.drone_mass, dt)
        accel = forces / self.drone_mass
        new_vel = current_vel + accel * dt
        new_vel *= 0.95  # Damping

        return new_vel

    def compute_dog_command(self, drone_pos: np.ndarray, dog_pos: np.ndarray, dog_vel: np.ndarray) -> np.ndarray:
        """Compute velocity command for dog using APF."""
        drone_2d = drone_pos[:2]
        dog_2d = dog_pos[:2]
        dog_vel_2d = dog_vel[:2] if len(dog_vel) >= 2 else np.array([0.0, 0.0])

        velocity_cmd = self.apf.compute_velocity(drone_2d, dog_2d, dog_vel_2d)

        # Return [vx, vy, yaw_rate] for the RL policy observation
        return np.array([velocity_cmd[0], velocity_cmd[1], 0.0])

    def log_state(self, drone_pos: np.ndarray, dog_pos: np.ndarray, dog_cmd: np.ndarray):
        """Log current state."""
        separation = np.linalg.norm(drone_pos[:2] - dog_pos[:2])
        progress = self.planner.get_progress(self.sim_time) * 100

        self.log_data['time'].append(self.sim_time)
        self.log_data['drone_pos'].append(drone_pos.copy())
        self.log_data['dog_pos'].append(dog_pos.copy())
        self.log_data['separation'].append(separation)
        self.log_data['dog_cmd'].append(dog_cmd.copy())
        self.log_data['progress'].append(progress)

        return separation, progress

    def is_complete(self) -> bool:
        """Check if coverage is complete."""
        return self.planner.is_complete(self.sim_time)

    def save_log(self):
        """Save logged data to file."""
        log_dir = "logs/simulation"
        os.makedirs(log_dir, exist_ok=True)
        filename = f"{log_dir}/leader_follower_rl_{datetime.now().strftime('%Y%m%d_%H%M%S')}.npz"

        np.savez(
            filename,
            time=np.array(self.log_data['time']),
            drone_pos=np.array(self.log_data['drone_pos']),
            dog_pos=np.array(self.log_data['dog_pos']),
            separation=np.array(self.log_data['separation']),
            progress=np.array(self.log_data['progress']),
        )
        print(f"\nLog saved to: {filename}")
        return filename


def run_leader_follower_rl():
    """Main simulation with RL locomotion policy."""

    print("=" * 60)
    print("Leader-Follower Simulation with RL Locomotion")
    print("=" * 60)

    # Initialize custom environment module's base_command dict
    custom_rl_env.base_command = {}

    # Import environment config
    from custom_rl_env import UnitreeGo2CustomEnvCfg

    # Create environment
    env_cfg = UnitreeGo2CustomEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs

    # Force flat terrain for leader-follower demo
    from omni.isaac.orbit.terrains import TerrainImporterCfg
    env_cfg.scene.terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        debug_vis=False,
    )

    # Create gymnasium environment
    env = gym.make(args_cli.task, cfg=env_cfg)
    env = RslRlVecEnvWrapper(env)

    # Initialize base command for each environment
    for i in range(env.unwrapped.num_envs):
        custom_rl_env.base_command[str(i)] = [0.0, 0.0, 0.0]

    # Load RL policy
    agent_cfg = agent_cfg_module.unitree_go2_agent_cfg
    log_root_path = os.path.join("logs", "rsl_rl", agent_cfg["experiment_name"])

    try:
        resume_path = get_checkpoint_path(log_root_path, agent_cfg["load_run"], agent_cfg["load_checkpoint"])
        print(f"Loading policy from: {resume_path}")

        ppo_runner = OnPolicyRunner(env, agent_cfg, log_dir=None, device=agent_cfg["device"])
        ppo_runner.load(resume_path)
        policy = ppo_runner.get_inference_policy(device=env.unwrapped.device)
    except Exception as e:
        print(f"Warning: Could not load policy ({e}). Using zero actions.")
        policy = None

    # Initialize leader-follower system
    lf_system = LeaderFollowerSystem(args_cli)

    # Get initial observations
    obs, _ = env.get_observations()

    # Simulation parameters
    dt = env.unwrapped.step_dt
    last_log_time = 0.0

    print(f"\nStarting simulation loop...")
    print(f"Step dt: {dt:.4f}s")
    print("-" * 60)

    # For drone, we need to spawn it separately in the scene
    # Since we're using the Go2 environment, we'll track the drone position externally
    # and use it for APF computation

    drone_pos = np.array([0.0, 0.0, args_cli.drone_altitude])
    drone_vel = np.array([0.0, 0.0, 0.0])

    while simulation_app.is_running():

        # Check completion
        if lf_system.is_complete():
            print("\n" + "=" * 60)
            print("COVERAGE COMPLETE!")
            print(f"Total time: {lf_system.sim_time:.1f}s")
            print(f"Min separation: {lf_system.apf.get_min_distance_observed():.2f}m")
            print(f"Safety maintained: {lf_system.apf.get_min_distance_observed() >= args_cli.d_safe}")
            print("=" * 60)
            break

        # --- DRONE UPDATE (simulated position tracking) ---
        target = lf_system.get_drone_target()
        error = target - drone_pos
        drone_vel = np.clip(error * 2.0, -args_cli.drone_speed, args_cli.drone_speed)
        drone_pos = drone_pos + drone_vel * dt

        # --- DOG CONTROL ---
        # Get dog position from environment
        robot = env.unwrapped.scene["robot"]
        dog_pos = robot.data.root_pos_w[0].cpu().numpy()
        dog_vel = robot.data.root_lin_vel_w[0].cpu().numpy()

        # Compute APF velocity command
        dog_cmd = lf_system.compute_dog_command(drone_pos, dog_pos, dog_vel)

        # Set velocity command for RL policy observation
        custom_rl_env.base_command["0"] = [dog_cmd[0], dog_cmd[1], dog_cmd[2]]

        # Run RL policy
        if policy is not None:
            with torch.inference_mode():
                actions = policy(obs)
        else:
            actions = torch.zeros((env.unwrapped.num_envs, env.unwrapped.num_actions),
                                  device=env.unwrapped.device)

        # Step environment
        obs, _, _, _ = env.step(actions)

        # --- LOGGING ---
        if lf_system.sim_time - last_log_time >= args_cli.log_interval:
            separation, progress = lf_system.log_state(drone_pos, dog_pos, dog_cmd)
            print(f"t={lf_system.sim_time:6.1f}s | "
                  f"Drone: ({drone_pos[0]:5.2f}, {drone_pos[1]:5.2f}, {drone_pos[2]:5.2f}) | "
                  f"Dog: ({dog_pos[0]:5.2f}, {dog_pos[1]:5.2f}) | "
                  f"Sep: {separation:4.2f}m | "
                  f"Cmd: ({dog_cmd[0]:4.2f}, {dog_cmd[1]:4.2f}) | "
                  f"Progress: {progress:5.1f}%")
            last_log_time = lf_system.sim_time

        lf_system.sim_time += dt

    # Save log and cleanup
    lf_system.save_log()
    env.close()


if __name__ == "__main__":
    run_leader_follower_rl()
