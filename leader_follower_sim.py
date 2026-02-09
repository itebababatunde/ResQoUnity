"""
Leader-Follower Simulation: Drone (Leader) + Go2 Dog (Follower)

This script implements a two-robot system where:
- Drone follows a lawnmower sweep trajectory for area coverage
- Go2 quadruped follows the drone using APF-based control

Usage:
    cd /home/kishi/IsaacLab_v0.3.1
    ./orbit.sh --sim /home/kishi/ResQoUnity/leader_follower_sim.py --terrain flat
"""

from __future__ import annotations

import argparse

# Parse arguments BEFORE any Isaac imports
parser = argparse.ArgumentParser(description="Leader-Follower Drone-Dog Simulation")
parser.add_argument("--cpu", action="store_true", default=False, help="Use CPU pipeline")
parser.add_argument("--disable_fabric", action="store_true", default=False, help="Disable fabric")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments")
parser.add_argument("--task", type=str, default="Isaac-Velocity-Rough-Unitree-Go2-v0", help="RL task")
parser.add_argument("--seed", type=int, default=None, help="Random seed")
parser.add_argument("--terrain", type=str, default="flat", help="Terrain type")
parser.add_argument("--robot", type=str, default="go2", help="Robot type")
parser.add_argument("--robot_amount", type=int, default=1, help="Robot amount")

# Leader-follower parameters
parser.add_argument("--workspace_size", type=float, default=10.0, help="Workspace side length (m)")
parser.add_argument("--sweep_spacing", type=float, default=2.0, help="Lawnmower row spacing (m)")
parser.add_argument("--drone_speed", type=float, default=0.8, help="Drone speed (m/s)")
parser.add_argument("--drone_altitude", type=float, default=3.0, help="Drone altitude (m)")
parser.add_argument("--d_safe", type=float, default=2.0, help="Minimum separation (m)")
parser.add_argument("--k_att", type=float, default=0.8, help="APF attractive gain")
parser.add_argument("--k_rep", type=float, default=2.0, help="APF repulsive gain")
parser.add_argument("--max_dog_velocity", type=float, default=1.2, help="Max dog velocity (m/s)")

# Import and add AppLauncher arguments
from omni.isaac.orbit.app import AppLauncher
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# Launch Isaac Sim BEFORE any other Isaac imports
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# ============================================================================
# Now import everything else AFTER Isaac Sim is launched
# ============================================================================

import math
import numpy as np
import torch
import gymnasium as gym
import os
import sys
from datetime import datetime
from dataclasses import MISSING
from typing import Literal

import carb
import omni.isaac.orbit.sim as sim_utils
from omni.isaac.orbit.envs import RLTaskEnvCfg
from omni.isaac.orbit.utils import configclass
from omni.isaac.orbit.assets import ArticulationCfg, AssetBaseCfg
from omni.isaac.orbit.scene import InteractiveSceneCfg
from omni.isaac.orbit.sensors import ContactSensorCfg, RayCasterCfg, patterns
from omni.isaac.orbit.terrains import TerrainImporterCfg
from omni.isaac.orbit.managers import EventTermCfg as EventTerm
from omni.isaac.orbit.managers import ObservationGroupCfg as ObsGroup
from omni.isaac.orbit.managers import ObservationTermCfg as ObsTerm
from omni.isaac.orbit.managers import RewardTermCfg as RewTerm
from omni.isaac.orbit.managers import SceneEntityCfg
from omni.isaac.orbit.managers import TerminationTermCfg as DoneTerm
from omni.isaac.orbit.utils.noise import AdditiveUniformNoiseCfg as Unoise
import omni.isaac.orbit_tasks.locomotion.velocity.mdp as mdp
from omni.isaac.orbit_assets.unitree import UNITREE_GO2_CFG
from omni.isaac.orbit_tasks.utils import get_checkpoint_path
from omni.isaac.orbit_tasks.utils.wrappers.rsl_rl import RslRlVecEnvWrapper, RslRlOnPolicyRunnerCfg
from rsl_rl.runners import OnPolicyRunner

# Local imports
from lawnmower_planner import LawnmowerPlanner, LawnmowerConfig
from apf_controller import APFController, APFConfig
from robots.quadcopter.config import QUADCOPTER_CFG

# ============================================================================
# Global state for velocity commands (used by observation function)
# ============================================================================
base_command = {"0": [0.0, 0.0, 0.0]}


def constant_commands(env: RLTaskEnvCfg) -> torch.Tensor:
    """Return per-env velocity commands [vx, vy, yaw_rate]."""
    commands = torch.zeros((env.num_envs, 3), dtype=torch.float32, device=env.device)
    for i in range(env.num_envs):
        key = str(i)
        if key in base_command:
            try:
                val = base_command[key]
                commands[i] = torch.as_tensor(val, dtype=torch.float32, device=env.device)
            except Exception:
                pass
    return commands


# ============================================================================
# Scene Configuration for Leader-Follower
# ============================================================================
@configclass
class LeaderFollowerSceneCfg(InteractiveSceneCfg):
    """Scene with flat terrain and Go2 robot."""

    # Flat ground
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        debug_vis=False,
    )

    # Go2 robot (follower)
    robot: ArticulationCfg = UNITREE_GO2_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # Height scanner
    height_scanner = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base",
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 20.0)),
        attach_yaw_only=True,
        pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=[1.6, 1.0]),
        debug_vis=False,
        mesh_prim_paths=["/World/ground"],
    )

    # Contact sensors
    contact_forces = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/.*",
        history_length=3,
        track_air_time=True
    )

    # Lighting
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DistantLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )


@configclass
class ViewerCfg:
    """Viewport camera configuration."""
    eye: tuple = (7.5, 7.5, 7.5)
    lookat: tuple = (0.0, 0.0, 0.0)
    cam_prim_path: str = "/OmniverseKit_Persp"
    resolution: tuple = (1920, 1080)
    origin_type: Literal["world", "env", "asset_root"] = "world"
    env_index: int = 0
    asset_name: str | None = None


@configclass
class ObservationsCfg:
    """Observation specifications."""
    @configclass
    class PolicyCfg(ObsGroup):
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel)
        projected_gravity = ObsTerm(func=mdp.projected_gravity, noise=Unoise(n_min=-0.05, n_max=0.05))
        velocity_commands = ObsTerm(func=constant_commands)
        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        actions = ObsTerm(func=mdp.last_action)
        height_scan = ObsTerm(
            func=mdp.height_scan,
            params={"sensor_cfg": SceneEntityCfg("height_scanner")},
            clip=(-1.0, 1.0),
        )

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class ActionsCfg:
    """Action specifications."""
    joint_pos = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=[".*"],
        scale=0.25,
        use_default_offset=True
    )


@configclass
class CommandsCfg:
    """Command specifications."""
    base_velocity = mdp.UniformVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(0.0, 0.0),
        rel_standing_envs=0.02,
        rel_heading_envs=1.0,
        heading_command=True,
        heading_control_stiffness=0.5,
        debug_vis=False,
        ranges=mdp.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(0.0, 0.0), lin_vel_y=(0.0, 0.0), ang_vel_z=(0.0, 0.0), heading=(0, 0)
        ),
    )


@configclass
class RewardsCfg:
    """Reward terms."""
    track_lin_vel_xy_exp = RewTerm(
        func=mdp.track_lin_vel_xy_exp, weight=1.5,
        params={"command_name": "base_velocity", "std": math.sqrt(0.25)}
    )
    track_ang_vel_z_exp = RewTerm(
        func=mdp.track_ang_vel_z_exp, weight=0.75,
        params={"command_name": "base_velocity", "std": math.sqrt(0.25)}
    )
    lin_vel_z_l2 = RewTerm(func=mdp.lin_vel_z_l2, weight=-2.0)
    ang_vel_xy_l2 = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.05)
    dof_torques_l2 = RewTerm(func=mdp.joint_torques_l2, weight=-0.0002)
    dof_acc_l2 = RewTerm(func=mdp.joint_acc_l2, weight=-2.5e-7)
    action_rate_l2 = RewTerm(func=mdp.action_rate_l2, weight=-0.01)
    feet_air_time = RewTerm(
        func=mdp.feet_air_time, weight=0.01,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot"),
            "command_name": "base_velocity",
            "threshold": 0.5,
        },
    )


@configclass
class TerminationsCfg:
    """Termination terms."""
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    base_contact = DoneTerm(
        func=mdp.illegal_contact,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names="base"), "threshold": 1.0},
    )


@configclass
class EventCfg:
    """Event configuration."""
    physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.8, 0.8),
            "dynamic_friction_range": (0.6, 0.6),
            "restitution_range": (0.0, 0.0),
            "num_buckets": 64,
        },
    )


@configclass
class LeaderFollowerEnvCfg(RLTaskEnvCfg):
    """Environment configuration for leader-follower."""

    scene: LeaderFollowerSceneCfg = LeaderFollowerSceneCfg(num_envs=1, env_spacing=2.5)
    viewer: ViewerCfg = ViewerCfg()
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()

    def __post_init__(self):
        self.decimation = 4
        self.sim.render_interval = self.decimation
        self.episode_length_s = 200.0  # Longer for full coverage
        self.sim.dt = 0.005
        self.sim.disable_contact_processing = True

        if self.scene.height_scanner is not None:
            self.scene.height_scanner.update_period = self.decimation * self.sim.dt
        if self.scene.contact_forces is not None:
            self.scene.contact_forces.update_period = self.sim.dt


# ============================================================================
# Leader-Follower Controller
# ============================================================================
class LeaderFollowerController:
    """Controls the leader-follower system."""

    def __init__(self, args):
        self.args = args

        self.planner = LawnmowerPlanner(LawnmowerConfig(
            workspace_size=args.workspace_size,
            sweep_spacing=args.sweep_spacing,
            drone_speed=args.drone_speed,
            drone_altitude=args.drone_altitude,
            start_corner=(0.0, 0.0),
        ))

        self.apf = APFController(APFConfig(
            k_att=args.k_att,
            k_rep=args.k_rep,
            d_safe=args.d_safe,
            max_velocity=args.max_dog_velocity,
        ))

        self.sim_time = 0.0
        self.drone_pos = np.array([0.0, 0.0, args.drone_altitude])
        self.drone_vel = np.array([0.0, 0.0, 0.0])
        self.log_data = []
        self.last_log_time = 0.0

        print(f"\n{'='*60}")
        print("Leader-Follower System Initialized")
        print(f"{'='*60}")
        print(f"Workspace: {args.workspace_size}m x {args.workspace_size}m")
        print(f"Drone: speed={args.drone_speed}m/s, altitude={args.drone_altitude}m")
        print(f"Sweep spacing: {args.sweep_spacing}m")
        print(f"Safety distance: {args.d_safe}m")
        print(f"APF: k_att={args.k_att}, k_rep={args.k_rep}")
        print(f"Expected duration: {self.planner.total_time:.1f}s")
        print(f"{'='*60}\n")

    def update_drone(self, dt):
        target = self.planner.get_position(self.sim_time)
        error = target - self.drone_pos
        self.drone_vel = np.clip(error * 2.0, -self.args.drone_speed, self.args.drone_speed)
        self.drone_pos = self.drone_pos + self.drone_vel * dt
        return self.drone_pos

    def compute_dog_command(self, dog_pos, dog_vel):
        drone_2d = self.drone_pos[:2]
        dog_2d = dog_pos[:2]
        dog_vel_2d = dog_vel[:2] if len(dog_vel) >= 2 else np.zeros(2)
        velocity_cmd = self.apf.compute_velocity(drone_2d, dog_2d, dog_vel_2d)
        return np.array([velocity_cmd[0], velocity_cmd[1], 0.0])

    def log_state(self, dog_pos, dog_cmd):
        separation = np.linalg.norm(self.drone_pos[:2] - dog_pos[:2])
        progress = self.planner.get_progress(self.sim_time) * 100

        if self.sim_time - self.last_log_time >= 1.0:
            print(f"t={self.sim_time:6.1f}s | "
                  f"Drone: ({self.drone_pos[0]:5.2f}, {self.drone_pos[1]:5.2f}) | "
                  f"Dog: ({dog_pos[0]:5.2f}, {dog_pos[1]:5.2f}) | "
                  f"Sep: {separation:4.2f}m | "
                  f"Progress: {progress:5.1f}%")
            self.last_log_time = self.sim_time
            self.log_data.append({
                'time': self.sim_time, 'separation': separation, 'progress': progress
            })

    def is_complete(self):
        return self.planner.is_complete(self.sim_time)


# ============================================================================
# Agent Configuration (from agent_cfg.py)
# ============================================================================
unitree_go2_agent_cfg = {
    'seed': 42,
    'device': 'cuda',
    'num_steps_per_env': 24,
    'max_iterations': 1500,
    'empirical_normalization': False,
    'policy': {
        'class_name': 'ActorCritic',
        'init_noise_std': 1.0,
        'actor_hidden_dims': [512, 256, 128],
        'critic_hidden_dims': [512, 256, 128],
        'activation': 'elu',
    },
    'algorithm': {
        'class_name': 'PPO',
        'value_loss_coef': 1.0,
        'use_clipped_value_loss': True,
        'clip_param': 0.2,
        'entropy_coef': 0.01,
        'num_learning_epochs': 5,
        'num_mini_batches': 4,
        'learning_rate': 0.001,
        'schedule': 'adaptive',
        'gamma': 0.99,
        'lam': 0.95,
        'desired_kl': 0.01,
        'max_grad_norm': 1.0,
    },
    'save_interval': 50,
    'experiment_name': 'unitree_go2_rough',
    'run_name': '',
    'logger': 'tensorboard',
    'neptune_project': 'orbit',
    'wandb_project': 'orbit',
    'resume': False,
    'load_run': -1,
    'load_checkpoint': -1,
}


# ============================================================================
# Main Function
# ============================================================================
def run_leader_follower():
    """Main simulation loop."""
    global base_command

    import omni.appwindow
    _input = carb.input.acquire_input_interface()
    _appwindow = omni.appwindow.get_default_app_window()

    # Create environment
    print("[INFO] Creating environment...")
    env_cfg = LeaderFollowerEnvCfg()
    env_cfg.scene.num_envs = 1

    env = gym.make(args_cli.task, cfg=env_cfg)
    env = RslRlVecEnvWrapper(env)
    print("[INFO] Environment created")

    # Load RL policy
    agent_cfg = dict(unitree_go2_agent_cfg)
    log_root_path = os.path.join("logs", "rsl_rl", agent_cfg["experiment_name"])
    log_root_path = os.path.abspath(log_root_path)

    try:
        resume_path = get_checkpoint_path(log_root_path, agent_cfg["load_run"], agent_cfg["load_checkpoint"])
        print(f"[INFO] Loading policy: {resume_path}")
        agent_cfg["obs_groups"] = {"policy": ["policy"]}
        ppo_runner = OnPolicyRunner(env, agent_cfg, log_dir=None, device=agent_cfg["device"])
        ppo_runner.load(resume_path)
        policy = ppo_runner.get_inference_policy(device=env.unwrapped.device)
        print("[INFO] Policy loaded")
    except Exception as e:
        print(f"[WARN] Could not load policy: {e}")
        print("[INFO] Using zero actions")
        num_actions = env.unwrapped.action_manager.action.shape[1]
        device = env.unwrapped.device
        policy = lambda obs: torch.zeros((1, num_actions), device=device)

    # Spawn drone visually
    print("[INFO] Spawning leader drone...")
    try:
        drone_spawn_pos = (0.0, 0.0, args_cli.drone_altitude)
        QUADCOPTER_CFG.spawn.func(
            "/World/LeaderDrone",
            QUADCOPTER_CFG.spawn,
            translation=drone_spawn_pos,
            orientation=(1.0, 0.0, 0.0, 0.0),
        )
        print("[INFO] Drone spawned at /World/LeaderDrone")
    except Exception as e:
        print(f"[WARN] Could not spawn drone: {e}")

    # Initialize controller
    lf = LeaderFollowerController(args_cli)

    # Get initial observations
    obs, _ = env.get_observations()
    dt = env.unwrapped.step_dt

    print(f"[INFO] Starting simulation (dt={dt:.4f}s)")
    print("-" * 60)

    # Main loop
    while simulation_app.is_running():
        if lf.is_complete():
            print("\n" + "=" * 60)
            print("COVERAGE COMPLETE!")
            print(f"Time: {lf.sim_time:.1f}s")
            print(f"Min separation: {lf.apf.get_min_distance_observed():.2f}m")
            print(f"Safety OK: {lf.apf.get_min_distance_observed() >= args_cli.d_safe}")
            print("=" * 60)
            break

        # Update drone
        lf.update_drone(dt)

        # Get dog state
        robot = env.unwrapped.scene["robot"]
        dog_pos = robot.data.root_pos_w[0].cpu().numpy()
        dog_vel = robot.data.root_lin_vel_w[0].cpu().numpy()

        # Compute APF command
        dog_cmd = lf.compute_dog_command(dog_pos, dog_vel)
        base_command["0"] = [dog_cmd[0], dog_cmd[1], dog_cmd[2]]

        # Run policy
        with torch.inference_mode():
            actions = policy(obs)

        # Step
        obs, _, _, _ = env.step(actions)
        lf.log_state(dog_pos, dog_cmd)
        lf.sim_time += dt

    env.close()


if __name__ == "__main__":
    run_leader_follower()
