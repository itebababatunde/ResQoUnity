"""Script to play a checkpoint if an RL agent from RSL-RL."""
from __future__ import annotations


"""Launch Isaac Sim Simulator first."""
import argparse
from omni.isaac.orbit.app import AppLauncher


import cli_args  
import time
import os
import threading


# add argparse arguments
parser = argparse.ArgumentParser(description="Train an RL agent with RSL-RL.")
parser.add_argument("--cpu", action="store_true", default=False, help="Use CPU pipeline.")
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default="Isaac-Velocity-Rough-Unitree-Go2-v0", help="Name of the task.")
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")
parser.add_argument("--custom_env", type=str, default="office", help="Setup the environment")
parser.add_argument("--robot", type=str, default="go2", help="Setup the robot")
parser.add_argument("--terrain", type=str, default="rough", help="Setup the robot")
parser.add_argument("--robot_amount", type=int, default=1, help="Setup the robot amount")


# append RSL-RL cli arguments
cli_args.add_rsl_rl_args(parser)


# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()


# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app


import omni


ext_manager = omni.kit.app.get_app().get_extension_manager()
ext_manager.set_extension_enabled_immediate("omni.isaac.ros2_bridge", True)

# FOR VR SUPPORT
# ext_manager.set_extension_enabled_immediate("omni.kit.xr.core", True)
# ext_manager.set_extension_enabled_immediate("omni.kit.xr.system.steamvr", True)
# ext_manager.set_extension_enabled_immediate("omni.kit.xr.system.simulatedxr", True)
# ext_manager.set_extension_enabled_immediate("omni.kit.xr.system.openxr", True)
# ext_manager.set_extension_enabled_immediate("omni.kit.xr.telemetry", True)
# ext_manager.set_extension_enabled_immediate("omni.kit.xr.profile.vr", True)


"""Rest everything follows."""
import gymnasium as gym
import torch
import carb


from omni.isaac.orbit_tasks.utils import get_checkpoint_path
from omni.isaac.orbit_tasks.utils.wrappers.rsl_rl import (
    RslRlOnPolicyRunnerCfg,
    RslRlVecEnvWrapper
)
import omni.isaac.orbit.sim as sim_utils
import omni.appwindow
from rsl_rl.runners import OnPolicyRunner



import rclpy
from ros2_bridge import RobotBaseNode, add_camera, add_rtx_lidar, pub_robo_data_ros2
from geometry_msgs.msg import Twist


import agent_cfg as agent_cfg_module
import custom_rl_env as custom_env_module
import custom_rl_env

from omnigraph import create_front_cam_omnigraph


class _RslRlEnvShim:
    """Shim to adapt env.get_observations() to return only observations for older rsl_rl versions.

    Some versions of rsl_rl expect env.get_observations() -> dict (no info tuple).
    IsaacLab/gymnasium returns (obs, info). This shim unwraps the tuple for the runner.
    """
    def __init__(self, env):
        self._env = env

    def __getattr__(self, name):
        return getattr(self._env, name)

    def get_observations(self):
        result = self._env.get_observations()
        if isinstance(result, tuple):
            return result[0]
        return result


def sub_keyboard_event(event, *args, **kwargs) -> bool:

    if len(custom_rl_env.base_command) > 0:
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input.name == 'W':
                custom_rl_env.base_command["0"] = [1, 0, 0]
            if event.input.name == 'S':
                custom_rl_env.base_command["0"] = [-1, 0, 0]
            if event.input.name == 'A':
                custom_rl_env.base_command["0"] = [0, 1, 0]
            if event.input.name == 'D':
                custom_rl_env.base_command["0"] = [0, -1, 0]
            if event.input.name == 'Q':
                custom_rl_env.base_command["0"] = [0, 0, 1]
            if event.input.name == 'E':
                custom_rl_env.base_command["0"] = [0, 0, -1]

            if len(custom_rl_env.base_command) > 1:
                if event.input.name == 'I':
                    custom_rl_env.base_command["1"] = [1, 0, 0]
                if event.input.name == 'K':
                    custom_rl_env.base_command["1"] = [-1, 0, 0]
                if event.input.name == 'J':
                    custom_rl_env.base_command["1"] = [0, 1, 0]
                if event.input.name == 'L':
                    custom_rl_env.base_command["1"] = [0, -1, 0]
                if event.input.name == 'U':
                    custom_rl_env.base_command["1"] = [0, 0, 1]
                if event.input.name == 'O':
                    custom_rl_env.base_command["1"] = [0, 0, -1]
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            for i in range(len(custom_rl_env.base_command)):
                custom_rl_env.base_command[str(i)] = [0, 0, 0]
    return True


def setup_custom_env():
    try:
        if (args_cli.custom_env == "warehouse" and args_cli.terrain == 'flat'):
            cfg_scene = sim_utils.UsdFileCfg(usd_path="./envs/warehouse.usd")
            cfg_scene.func("/World/warehouse", cfg_scene, translation=(0.0, 0.0, 0.0))

        if (args_cli.custom_env == "office" and args_cli.terrain == 'flat'):
            cfg_scene = sim_utils.UsdFileCfg(usd_path="./envs/office.usd")
            cfg_scene.func("/World/office", cfg_scene, translation=(0.0, 0.0, 0.0))
    except:
        print("Error loading custom environment. You should download custom envs folder from: https://drive.google.com/drive/folders/1vVGuO1KIX1K6mD6mBHDZGm9nk2vaRyj3?usp=sharing")


def cmd_vel_cb(msg, num_robot):
    x = msg.linear.x
    y = msg.linear.y
    z = msg.angular.z
    _ensure_base_command_dict()
    custom_rl_env.base_command[str(num_robot)] = [x, y, z]



def add_cmd_sub(num_envs):
    node_test = rclpy.create_node('position_velocity_publisher')
    for i in range(num_envs):
        node_test.create_subscription(Twist, f'robot{i}/cmd_vel', lambda msg, i=i: cmd_vel_cb(msg, str(i)), 10)
    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node_test,), daemon=True)
    thread.start()
def _ensure_base_command_dict():
    """Ensure base_command is a dict keyed by string indices.

    Some environments may accidentally set it to a list. Convert to dict if so.
    """
    if not isinstance(custom_rl_env.base_command, dict):
        try:
            seq = list(custom_rl_env.base_command)
            custom_rl_env.base_command = {str(i): list(seq[i]) for i in range(len(seq))}
        except Exception:
            custom_rl_env.base_command = {}




def specify_cmd_for_robots(numv_envs):
    _ensure_base_command_dict()
    for i in range(numv_envs):
        custom_rl_env.base_command[str(i)] = [0, 0, 0]
def run_sim():
    
    # acquire input interface
    _input = carb.input.acquire_input_interface()
    _appwindow = omni.appwindow.get_default_app_window()
    _keyboard = _appwindow.get_keyboard()
    _sub_keyboard = _input.subscribe_to_keyboard_events(_keyboard, sub_keyboard_event)

    """Play with RSL-RL agent."""
    # parse configuration
    
    # choose env cfg based on robot type, fall back if G1 is unavailable
    env_cfg = custom_env_module.UnitreeGo2CustomEnvCfg()
    if args_cli.robot == "g1":
        if hasattr(custom_env_module, "G1RoughEnvCfg"):
            env_cfg = custom_env_module.G1RoughEnvCfg()
        else:
            print("[WARN] G1RoughEnvCfg not found. Falling back to UnitreeGo2CustomEnvCfg.")

    # add N robots to env 
    env_cfg.scene.num_envs = args_cli.robot_amount

    # create ros2 camera stream omnigraph
    for i in range(env_cfg.scene.num_envs):
        create_front_cam_omnigraph(i)
        
    specify_cmd_for_robots(env_cfg.scene.num_envs)

    # copy to avoid mutating the imported dict
    agent_cfg_data: RslRlOnPolicyRunnerCfg = dict(agent_cfg_module.unitree_go2_agent_cfg)

    if args_cli.robot == "g1" and hasattr(agent_cfg_module, "unitree_g1_agent_cfg"):
        agent_cfg_data = dict(agent_cfg_module.unitree_g1_agent_cfg)

    # create isaac environment
    env = gym.make(args_cli.task, cfg=env_cfg)
    # wrap around environment for rsl-rl
    env = RslRlVecEnvWrapper(env)
    # specify directory for logging experiments
    log_root_path = os.path.join("logs", "rsl_rl", agent_cfg_data["experiment_name"])
    log_root_path = os.path.abspath(log_root_path)
    print(f"[INFO] Loading experiment from directory: {log_root_path}")

    resume_path = get_checkpoint_path(log_root_path, agent_cfg_data["load_run"], agent_cfg_data["load_checkpoint"])
    print(f"[INFO]: Loading model checkpoint from: {resume_path}")

    # ensure obs_groups is defined for rsl_rl runner
    if "obs_groups" not in agent_cfg_data:
        agent_cfg_data["obs_groups"] = {"policy": ["policy"]}

    # load previously trained model (shimmed env for rsl_rl)
    env_for_runner = _RslRlEnvShim(env)
    ppo_runner = OnPolicyRunner(env_for_runner, agent_cfg_data, log_dir=None, device=agent_cfg_data["device"])
    ppo_runner.load(resume_path)
    print(f"[INFO]: Loading model checkpoint from: {resume_path}")

    # obtain the trained policy for inference
    policy = ppo_runner.get_inference_policy(device=env.unwrapped.device)

    # reset environment
    obs, _ = env.get_observations()

    # initialize ROS2 node
    rclpy.init()
    base_node = RobotBaseNode(env_cfg.scene.num_envs)
    add_cmd_sub(env_cfg.scene.num_envs)

    annotator_lst = add_rtx_lidar(env_cfg.scene.num_envs, args_cli.robot, False)
    add_camera(env_cfg.scene.num_envs, args_cli.robot)
    setup_custom_env()
    
    start_time = time.time()
    # simulate environment
    while simulation_app.is_running():
        # run everything in inference mode
        with torch.inference_mode():
            # agent stepping
            actions = policy(obs)
            # env stepping
            obs, _, _, _ = env.step(actions)
            pub_robo_data_ros2(args_cli.robot, env_cfg.scene.num_envs, base_node, env, annotator_lst, start_time)
    env.close()