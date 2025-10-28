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
parser.add_argument("--robot_amount", type=int, default=2, help="Setup the robot amount")


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
import numpy as np
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
            result = result[0]
        # Ensure rsl_rl gets a dict when obs_groups are defined
        if isinstance(result, dict):
            return result
        return {"policy": result}


 


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
            # Drone altitude controls (T for up, G for down)
            # Store altitude command separately for Z-axis control
            if event.input.name == 'T':
                # Move up (positive Z velocity) - for drone mode
                if args_cli.robot == "drone" or args_cli.robot == "quadcopter":
                    custom_rl_env.drone_altitude = 1.0  # Altitude up
            if event.input.name == 'G':
                # Move down (negative Z velocity) - for drone mode
                if args_cli.robot == "drone" or args_cli.robot == "quadcopter":
                    custom_rl_env.drone_altitude = -1.0  # Altitude down

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
                # Drone altitude controls for robot1 (Y for up, H for down)
                if event.input.name == 'Y':
                    if args_cli.robot == "drone" or args_cli.robot == "quadcopter":
                        if not hasattr(custom_rl_env, 'drone_altitude_1'):
                            custom_rl_env.drone_altitude_1 = 0.0
                        custom_rl_env.drone_altitude_1 = 1.0  # Altitude up
                if event.input.name == 'H':
                    if args_cli.robot == "drone" or args_cli.robot == "quadcopter":
                        if not hasattr(custom_rl_env, 'drone_altitude_1'):
                            custom_rl_env.drone_altitude_1 = 0.0
                        custom_rl_env.drone_altitude_1 = -1.0  # Altitude down
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            for i in range(len(custom_rl_env.base_command)):
                custom_rl_env.base_command[str(i)] = [0, 0, 0]
            # Reset altitude commands for drones
            custom_rl_env.drone_altitude = 0.0
            if hasattr(custom_rl_env, 'drone_altitude_1'):
                custom_rl_env.drone_altitude_1 = 0.0
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
    """Handle velocity commands - only active in VELOCITY or ALTITUDE_HOLD modes"""
    # Check if drone is in velocity-accepting mode
    if args_cli.robot == "drone" or args_cli.robot == "quadcopter":
        controller = custom_rl_env.drone_controllers.get(str(num_robot))
        if controller:
            # Only accept velocity commands in these modes
            if controller.mode.value not in ['VELOCITY', 'ALTITUDE_HOLD', 'IDLE']:
                # Ignore velocity commands in position/loiter/landing modes
                print(f"[Drone {num_robot}] Ignoring cmd_vel in mode {controller.mode.value}")
                return
            
            # Set mode to VELOCITY when receiving cmd_vel
            if controller.mode.value == 'IDLE' or controller.mode.value == 'ALTITUDE_HOLD':
                from drone_controller import DroneState
                controller.set_mode(DroneState.VELOCITY)
    
    x = msg.linear.x
    y = msg.linear.y
    
    # For drones: use linear.z (altitude), for ground robots: use angular.z (yaw)
    if args_cli.robot == "drone" or args_cli.robot == "quadcopter":
        z = msg.angular.z  # Yaw for XY plane rotation
        # Store altitude separately for drones
        if num_robot == "0":
            custom_rl_env.drone_altitude = msg.linear.z
        elif num_robot == "1":
            if not hasattr(custom_rl_env, 'drone_altitude_1'):
                custom_rl_env.drone_altitude_1 = 0.0
            custom_rl_env.drone_altitude_1 = msg.linear.z
    else:
        z = msg.angular.z  # Yaw for ground robots
    
    _ensure_base_command_dict()
    custom_rl_env.base_command[str(num_robot)] = [x, y, z]



def cmd_position_cb(msg, num_robot):
    """Handle position setpoint commands"""
    from drone_controller import DroneState
    controller = custom_rl_env.drone_controllers.get(str(num_robot))
    if controller and controller.armed:
        target_x = msg.pose.position.x
        target_y = msg.pose.position.y
        target_z = msg.pose.position.z
        controller.set_target_position(target_x, target_y, target_z)
        controller.set_mode(DroneState.POSITION)
    else:
        if not controller:
            print(f"[Drone {num_robot}] No controller found for position command")
        else:
            print(f"[Drone {num_robot}] Drone not armed, ignoring position command")


def cmd_altitude_cb(msg, num_robot):
    """Handle altitude setpoint commands"""
    from drone_controller import DroneState
    controller = custom_rl_env.drone_controllers.get(str(num_robot))
    if controller and controller.armed:
        controller.set_target_altitude(msg.data)
        controller.set_mode(DroneState.ALTITUDE_HOLD)
    else:
        if not controller:
            print(f"[Drone {num_robot}] No controller found for altitude command")
        else:
            print(f"[Drone {num_robot}] Drone not armed, ignoring altitude command")


def takeoff_service_cb(request, response, num_robot):
    """Takeoff to default altitude (1.5m)"""
    from drone_controller import DroneState
    controller = custom_rl_env.drone_controllers.get(str(num_robot))
    
    if not controller:
        response.success = False
        response.message = f"Controller not initialized for robot{num_robot}"
        return response
    
    if not controller.armed:
        response.success = False
        response.message = "Drone is not armed. Call /arm service first."
        return response
    
    if controller.mode != DroneState.IDLE:
        response.success = False
        response.message = f"Cannot takeoff from mode {controller.mode.value}"
        return response
    
    # Set target altitude and switch to position mode
    current_pos = controller.current_position
    controller.set_target_position(current_pos[0], current_pos[1], 1.5)  # 1.5m altitude
    controller.set_mode(DroneState.POSITION)
    
    response.success = True
    response.message = "Takeoff initiated to 1.5m"
    return response


def land_service_cb(request, response, num_robot):
    """Initiate landing sequence"""
    from drone_controller import DroneState
    controller = custom_rl_env.drone_controllers.get(str(num_robot))
    
    if not controller or not controller.armed:
        response.success = False
        response.message = "Drone not armed or controller missing"
        return response
    
    controller.set_mode(DroneState.LANDING)
    response.success = True
    response.message = "Landing sequence initiated"
    return response


def emergency_stop_cb(request, response, num_robot):
    """Emergency stop - zero all velocities immediately"""
    from drone_controller import DroneState
    controller = custom_rl_env.drone_controllers.get(str(num_robot))
    
    if controller:
        controller.set_mode(DroneState.EMERGENCY)
        custom_rl_env.base_command[str(num_robot)] = [0, 0, 0]
        if num_robot == "0":
            custom_rl_env.drone_altitude = 0.0
        elif num_robot == "1":
            custom_rl_env.drone_altitude_1 = 0.0
    
    response.success = True
    response.message = "Emergency stop activated"
    return response


def arm_service_cb(request, response, num_robot):
    """Arm or disarm the drone"""
    from drone_controller import DroneState
    controller = custom_rl_env.drone_controllers.get(str(num_robot))
    
    if not controller:
        response.success = False
        response.message = "Controller not found"
        return response
    
    controller.armed = request.data
    controller.set_mode(DroneState.IDLE if request.data else DroneState.DISARMED)
    
    response.success = True
    response.message = f"Drone {'armed' if request.data else 'disarmed'}"
    return response


def add_cmd_sub(num_envs):
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Float32
    from std_srvs.srv import Trigger, SetBool
    
    node_test = rclpy.create_node('drone_control_node')
    for i in range(num_envs):
        # Velocity commands (all robots)
        node_test.create_subscription(Twist, f'robot{i}/cmd_vel', lambda msg, i=i: cmd_vel_cb(msg, str(i)), 10)
        
        # Position and altitude commands (drones only)
        if args_cli.robot == "drone" or args_cli.robot == "quadcopter":
            node_test.create_subscription(PoseStamped, f'robot{i}/cmd_position',
                                         lambda msg, i=i: cmd_position_cb(msg, str(i)), 10)
            node_test.create_subscription(Float32, f'robot{i}/cmd_altitude',
                                         lambda msg, i=i: cmd_altitude_cb(msg, str(i)), 10)
            
            # Services
            node_test.create_service(Trigger, f'robot{i}/takeoff',
                                    lambda req, res, i=i: takeoff_service_cb(req, res, str(i)))
            node_test.create_service(Trigger, f'robot{i}/land',
                                    lambda req, res, i=i: land_service_cb(req, res, str(i)))
            node_test.create_service(Trigger, f'robot{i}/emergency_stop',
                                    lambda req, res, i=i: emergency_stop_cb(req, res, str(i)))
            node_test.create_service(SetBool, f'robot{i}/arm',
                                    lambda req, res, i=i: arm_service_cb(req, res, str(i)))
    
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
    elif args_cli.robot == "drone" or args_cli.robot == "quadcopter":
        if hasattr(custom_env_module, "QuadcopterEnvCfg"):
            env_cfg = custom_env_module.QuadcopterEnvCfg()
        else:
            print("[WARN] QuadcopterEnvCfg not found. Falling back to UnitreeGo2CustomEnvCfg.")

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
    elif (args_cli.robot == "drone" or args_cli.robot == "quadcopter") and hasattr(agent_cfg_module, "quadcopter_agent_cfg"):
        agent_cfg_data = dict(agent_cfg_module.quadcopter_agent_cfg)

    # create isaac environment
    print(f"[INFO] Creating environment: {args_cli.task}")
    print(f"[INFO] Robot: {args_cli.robot}, Amount: {args_cli.robot_amount}, Terrain: {args_cli.terrain}")
    env = gym.make(args_cli.task, cfg=env_cfg)
    print("[INFO] Environment created successfully!")
    # wrap around environment for rsl-rl
    env = RslRlVecEnvWrapper(env)
    print("[INFO] Environment wrapped for RSL-RL")
    # specify directory for logging experiments
    log_root_path = os.path.join("logs", "rsl_rl", agent_cfg_data["experiment_name"])
    log_root_path = os.path.abspath(log_root_path)
    print(f"[INFO] Loading experiment from directory: {log_root_path}")

    # Check if checkpoint exists (for drones in testing mode, may not have trained policy)
    use_trained_policy = os.path.exists(log_root_path)
    
    if use_trained_policy:
        resume_path = get_checkpoint_path(log_root_path, agent_cfg_data["load_run"], agent_cfg_data["load_checkpoint"])
        print(f"[INFO]: Loading model checkpoint from: {resume_path}")

        # ensure obs_groups is a dict with a default "policy" group to match the shimmed observations
        agent_cfg_data["obs_groups"] = {"policy": ["policy"]}

        # load previously trained model (shimmed env for rsl_rl)
        env_for_runner = _RslRlEnvShim(env)
        ppo_runner = OnPolicyRunner(env_for_runner, agent_cfg_data, log_dir=None, device=agent_cfg_data["device"])
        ppo_runner.load(resume_path)
        print(f"[INFO]: Checkpoint loaded successfully")

        # obtain the trained policy for inference
        policy = ppo_runner.get_inference_policy(device=env.unwrapped.device)
    else:
        print(f"[WARN] No checkpoint found at {log_root_path}")
        print(f"[INFO] Running in MANUAL CONTROL mode (keyboard only, no RL policy)")
        # Create a velocity-based controller for drone
        # Get action space size from environment
        num_envs = env.unwrapped.num_envs
        num_actions = env.unwrapped.action_manager.action.shape[1]
        device = env.unwrapped.device
        
        def drone_velocity_controller(obs_dict):
            # For drones: we'll apply velocities directly in the main loop
            # Still return zero actions (we bypass motor control)
            return torch.zeros((num_envs, num_actions), device=device)
        
        # Policy must return shape (num_envs, num_actions)
        policy = drone_velocity_controller

    # Initialize drone controllers (if using drones)
    if args_cli.robot == "drone" or args_cli.robot == "quadcopter":
        from drone_controller import DroneController
        print("[INFO] Initializing drone controllers...")
        
        for i in range(env.unwrapped.num_envs):
            custom_rl_env.drone_controllers[str(i)] = DroneController(
                robot_id=str(i),
                kp_pos=0.8,      # Position PID gains
                ki_pos=0.01,
                kd_pos=0.4,
                kp_alt=1.5,      # Altitude PID gains
                ki_alt=0.05,
                kd_alt=0.8,
                kp_att=2.0,      # Attitude PID gains
                ki_att=0.0,
                kd_att=0.5,
                kp_yaw=1.0,      # Yaw rate PID gains
                ki_yaw=0.0,
                kd_yaw=0.2,
                max_vel=2.0,
                max_climb_rate=1.5,
                max_angle_rate=2.0,    # Max roll/pitch rate (rad/s)
                max_yaw_rate=1.0,      # Max yaw rate (rad/s)
                hover_thrust=0.55      # Hover thrust (tunable per drone mass)
            )
            custom_rl_env.drone_armed[str(i)] = False
            custom_rl_env.drone_mode[str(i)] = 'DISARMED'
            print(f"[INFO] Drone {i} controller initialized in DISARMED mode (motor-based control)")
    
    # reset environment
    print("[INFO] Resetting environment and getting initial observations...")
    obs, _ = env.get_observations()
    if not isinstance(obs, dict):
        obs = {"policy": obs}
    print("[INFO] Environment reset complete, starting simulation loop...")

    # initialize ROS2 node
    rclpy.init()
    base_node = RobotBaseNode(env_cfg.scene.num_envs)
    add_cmd_sub(env_cfg.scene.num_envs)

    annotator_lst = add_rtx_lidar(env_cfg.scene.num_envs, args_cli.robot, False)
    add_camera(env_cfg.scene.num_envs, args_cli.robot)
    setup_custom_env()
    
    # Set up viewport camera to see the robot
    print("[INFO] Setting up viewport camera...")
    try:
        from omni.isaac.core.utils.viewports import set_camera_view
        # Position camera to look at the robot from a good angle
        # eye: camera position, target: what camera looks at
        set_camera_view(eye=[3.0, 3.0, 2.0], target=[0.0, 0.0, 0.5], camera_prim_path="/OmniverseKit_Persp")
        print("[INFO] Camera positioned to view robot")
    except Exception as e:
        print(f"[WARN] Could not set camera view: {e}")
    
    print(f"[INFO] Robot(s) should now be visible in the viewport!")
    print(f"[INFO] Use these controls to navigate:")
    print(f"  - Press 'F' to focus on robot")
    print(f"  - Mouse wheel: Zoom in/out")
    print(f"  - Middle mouse: Pan camera")
    print(f"  - Alt+Left mouse: Orbit view")
    print(f"  - WASD keys: Move robot (forward/back/left/right)")
    print(f"  - Q/E keys: Rotate left/right")
    if args_cli.robot == "drone" or args_cli.robot == "quadcopter":
        print(f"  - T/G keys: Altitude up/down (for drone)")
    
    # Note: Drone is now spawned by the environment configuration (not statically added)
    # If robot type is 'drone' or 'quadcopter', the QuadcopterEnvCfg handles spawning
    
    start_time = time.time()
    # simulate environment
    while simulation_app.is_running():
        # run everything in inference mode
        with torch.inference_mode():
            # agent stepping
            actions = policy(obs)
            
            # Motor-based control for drones (uses physics)
            if args_cli.robot == "drone" or args_cli.robot == "quadcopter":
                # Access the robot articulation
                robot = env.unwrapped.scene["robot"]
                num_envs = env.unwrapped.num_envs
                
                # Build motor commands for all environments
                all_motor_commands = []
                
                for env_idx in range(num_envs):
                    controller = custom_rl_env.drone_controllers.get(str(env_idx))
                    
                    if controller and controller.armed:
                        # Get current state from environment
                        current_pos = robot.data.root_pos_w[env_idx].cpu().numpy()
                        current_vel = robot.data.root_lin_vel_w[env_idx].cpu().numpy()
                        current_quat = robot.data.root_quat_w[env_idx].cpu().numpy()  # [w,x,y,z]
                        
                        # Update controller state
                        dt = 1.0 / 60.0
                        controller.current_orientation = current_quat
                        controller.current_euler = controller._quat_to_euler(current_quat)
                        controller.update(dt, current_pos, current_vel)
                        
                        # Get motor commands from controller
                        motor_cmds = controller.compute_motor_command()
                        
                        # For VELOCITY mode, convert velocity inputs to motor commands
                        if controller.mode.value == 'VELOCITY':
                            # Get velocity commands from cmd_vel/keyboard
                            cmd = custom_rl_env.base_command.get(str(env_idx), [0, 0, 0])
                            if env_idx == 0:
                                altitude_cmd = getattr(custom_rl_env, 'drone_altitude', 0.0)
                            elif env_idx == 1:
                                altitude_cmd = getattr(custom_rl_env, 'drone_altitude_1', 0.0)
                            else:
                                altitude_cmd = 0.0
                            
                            # Convert velocities to motor commands (simplified)
                            vx, vy, vz, yaw_rate = cmd[0], cmd[1], altitude_cmd, cmd[2]
                            
                            desired_pitch = -vx * 0.3
                            desired_roll = vy * 0.3
                            thrust = controller.hover_thrust + vz * 0.2
                            thrust = np.clip(thrust, 0.0, 1.0)
                            
                            # Simple mapping
                            motor_cmds = controller._motor_mixer(thrust, desired_roll, desired_pitch, yaw_rate * 0.1)
                    
                    else:
                        # Not armed or no controller: zero motors
                        motor_cmds = np.zeros(4)
                    
                    all_motor_commands.append(motor_cmds)
                
                # Convert to tensor for action manager
                # Action shape: (num_envs, 4) for 4 motors
                motor_tensor = torch.tensor(all_motor_commands, device=robot.device, dtype=torch.float32)
                
                # Override the policy actions with motor commands
                actions = motor_tensor
            
            # env stepping
            obs, _, _, _ = env.step(actions)
            if not isinstance(obs, dict):
                obs = {"policy": obs}
            pub_robo_data_ros2(args_cli.robot, env_cfg.scene.num_envs, base_node, env, annotator_lst, start_time)
    env.close()