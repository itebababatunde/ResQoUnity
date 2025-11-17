"""Script to play a checkpoint if an RL agent from RSL-RL."""
from __future__ import annotations


"""Launch Isaac Sim Simulator first."""
import argparse
from omni.isaac.orbit.app import AppLauncher


import cli_args  
import time
import os
import threading
import sys
from datetime import datetime


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
from robots.quadcopter.config import QUADCOPTER_CFG


def calculate_drone_forces(desired_velocity, current_velocity, mass, dt=0.016, gravity=9.81):
    """
    Calculate forces needed to achieve desired velocity.
    Inspired by OmniDrones' approach but adapted for direct velocity control.
    
    Args:
        desired_velocity: np.array([vx, vy, vz]) - desired velocity in world frame (m/s)
        current_velocity: np.array([vx, vy, vz]) - current velocity in world frame (m/s)
        mass: float - drone mass in kg
        dt: float - time step (default 60 Hz = 0.016s)
        gravity: float - gravity constant (m/s^2)
    
    Returns:
        forces: np.array([fx, fy, fz]) - forces to apply in world frame (N)
    """
    # Calculate required acceleration to reach desired velocity
    # Using simple P controller: a = Kp * (v_desired - v_current)
    velocity_error = desired_velocity - current_velocity
    
    # Gain for acceleration controller (how aggressively to change velocity)
    # Higher = faster response but more oscillation
    kp_accel = 8.0  # Tuned for responsive but stable control
    
    desired_accel = kp_accel * velocity_error
    
    # Clamp acceleration to reasonable limits (prevent extreme forces)
    max_accel = 15.0  # m/s^2 (about 1.5g)
    desired_accel = np.clip(desired_accel, -max_accel, max_accel)
    
    # Convert acceleration to force: F = ma
    force = mass * desired_accel
    
    # CRITICAL: Add gravity compensation for vertical force
    # The drone must counteract gravity to maintain altitude
    force[2] += mass * gravity  # Add upward force to counteract gravity
    
    return force


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
    
    # Allow takeoff from IDLE or LOITER modes
    if controller.mode not in [DroneState.IDLE, DroneState.LOITER]:
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
    
    # Only change mode if arming state actually changes
    was_armed = controller.armed
    controller.armed = request.data
    
    if request.data and not was_armed:
        # Arming from disarmed: switch to IDLE
        controller.set_mode(DroneState.IDLE)
    elif not request.data:
        # Disarming: switch to DISARMED
        controller.set_mode(DroneState.DISARMED)
    # else: Already armed, keep current mode (LOITER/POSITION/etc)
    
    response.success = True
    response.message = f"Drone {'armed' if request.data else 'disarmed'}"
    return response


# ============= World-Level Drone Callbacks (for /World/Drone) =============

def world_drone_cmd_vel_cb(msg):
    """Handle velocity commands for world-level drone (/drone namespace)
    THREAD-SAFE: Uses world_drone_lock for concurrent access"""
    with custom_rl_env.world_drone_lock:
        controller = custom_rl_env.world_drone_controller
        if controller:
            # Only accept velocity commands in appropriate modes
            if controller.mode.value not in ['VELOCITY', 'ALTITUDE_HOLD', 'IDLE']:
                print(f"[Drone] Ignoring cmd_vel in mode {controller.mode.value}")
                return
            
            # Set mode to VELOCITY when receiving cmd_vel
            if controller.mode.value == 'IDLE' or controller.mode.value == 'ALTITUDE_HOLD':
                from drone_controller import DroneState
                controller.set_mode(DroneState.VELOCITY)
        
        # Store velocity commands (thread-safe under lock)
        custom_rl_env.world_drone_command = [msg.linear.x, msg.linear.y, msg.angular.z]
        custom_rl_env.world_drone_altitude = msg.linear.z


def world_drone_cmd_position_cb(msg):
    """Handle position setpoint commands for world-level drone
    THREAD-SAFE: Uses world_drone_lock for concurrent access"""
    from drone_controller import DroneState
    with custom_rl_env.world_drone_lock:
        controller = custom_rl_env.world_drone_controller
        if controller and controller.armed:
            target_x = msg.pose.position.x
            target_y = msg.pose.position.y
            target_z = msg.pose.position.z
            controller.set_target_position(target_x, target_y, target_z)
            controller.set_mode(DroneState.POSITION)
        else:
            if not controller:
                print(f"[Drone] No controller found for position command")
            else:
                print(f"[Drone] Drone not armed, ignoring position command")


def world_drone_cmd_altitude_cb(msg):
    """Handle altitude setpoint commands for world-level drone
    THREAD-SAFE: Uses world_drone_lock for concurrent access"""
    from drone_controller import DroneState
    with custom_rl_env.world_drone_lock:
        controller = custom_rl_env.world_drone_controller
        if controller and controller.armed:
            controller.set_target_altitude(msg.data)
            controller.set_mode(DroneState.ALTITUDE_HOLD)
        else:
            if not controller:
                print(f"[Drone] No controller found for altitude command")
            else:
                print(f"[Drone] Drone not armed, ignoring altitude command")


def world_drone_takeoff_cb(request, response):
    """Takeoff to default altitude (1.5m) for world-level drone
    THREAD-SAFE: Uses world_drone_lock for concurrent access"""
    from drone_controller import DroneState
    with custom_rl_env.world_drone_lock:
        controller = custom_rl_env.world_drone_controller
        
        if not controller:
            response.success = False
            response.message = "Controller not initialized for world drone"
            return response
        
        if not controller.armed:
            response.success = False
            response.message = "Drone is not armed. Call /drone/arm service first."
            return response
        
        # Allow takeoff from IDLE or LOITER modes
        if controller.mode not in [DroneState.IDLE, DroneState.LOITER]:
            response.success = False
            response.message = f"Cannot takeoff from mode {controller.mode.value}"
            return response
        
        # Set target altitude and switch to position mode
        current_pos = controller.current_position
        controller.set_target_position(current_pos[0], current_pos[1], 1.5)  # 1.5m altitude
        controller.set_mode(DroneState.POSITION)
        
        # Safety logging: Confirm takeoff state
        print(f"[SAFETY] Takeoff service executed:")
        print(f"  Mode: {controller.mode.value}")
        print(f"  Target: ({controller.target_position[0]:.3f}, {controller.target_position[1]:.3f}, {controller.target_position[2]:.3f})")
        print(f"  Current: ({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f})")
        
        response.success = True
        response.message = "Takeoff initiated to 1.5m"
        return response


def world_drone_land_cb(request, response):
    """Initiate landing sequence for world-level drone
    THREAD-SAFE: Uses world_drone_lock for concurrent access"""
    from drone_controller import DroneState
    with custom_rl_env.world_drone_lock:
        controller = custom_rl_env.world_drone_controller
        
        if not controller or not controller.armed:
            response.success = False
            response.message = "Drone not armed or controller missing"
            return response
        
        controller.set_mode(DroneState.LANDING)
        response.success = True
        response.message = "Landing sequence initiated"
        return response


def world_drone_emergency_stop_cb(request, response):
    """Emergency stop for world-level drone - zero all velocities immediately
    THREAD-SAFE: Uses world_drone_lock for concurrent access"""
    from drone_controller import DroneState
    with custom_rl_env.world_drone_lock:
        controller = custom_rl_env.world_drone_controller
        
        if controller:
            controller.set_mode(DroneState.EMERGENCY)
            custom_rl_env.world_drone_command = [0, 0, 0]
            custom_rl_env.world_drone_altitude = 0.0
        
        response.success = True
        response.message = "Emergency stop activated"
        return response


def world_drone_arm_cb(request, response):
    """Arm or disarm the world-level drone
    THREAD-SAFE: Uses world_drone_lock for concurrent access"""
    from drone_controller import DroneState
    with custom_rl_env.world_drone_lock:
        controller = custom_rl_env.world_drone_controller
        logger = custom_rl_env.world_drone_logger if hasattr(custom_rl_env, 'world_drone_logger') else None
        
        if not controller:
            response.success = False
            response.message = "Controller not found"
            return response
        
        # Only change mode if arming state actually changes
        was_armed = controller.armed
        controller.armed = request.data
        
        if request.data and not was_armed:
            # Arming from disarmed: switch to IDLE
            controller.set_mode(DroneState.IDLE)
        elif not request.data:
            # Disarming: switch to DISARMED
            controller.set_mode(DroneState.DISARMED)
        # else: Already armed, keep current mode (LOITER/POSITION/etc)
        
        # Log arm/disarm event
        if logger:
            logger.log_arm_event(request.data)
        
        response.success = True
        response.message = f"Drone {'armed' if request.data else 'disarmed'}"
        return response


def add_cmd_sub(num_envs, enable_world_drone=False):
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
    
    # Add world-level drone control (separate from robot0...robotN)
    if enable_world_drone:
        print("[INFO] Adding /drone namespace ROS2 interface")
        try:
        # Topics
        node_test.create_subscription(Twist, '/drone/cmd_vel', world_drone_cmd_vel_cb, 10)
        node_test.create_subscription(PoseStamped, '/drone/cmd_position', world_drone_cmd_position_cb, 10)
        node_test.create_subscription(Float32, '/drone/cmd_altitude', world_drone_cmd_altitude_cb, 10)
            print("[INFO] Drone topics created: cmd_vel, cmd_position, cmd_altitude")
        
        # Services
            arm_srv = node_test.create_service(SetBool, '/drone/arm', world_drone_arm_cb)
            takeoff_srv = node_test.create_service(Trigger, '/drone/takeoff', world_drone_takeoff_cb)
            land_srv = node_test.create_service(Trigger, '/drone/land', world_drone_land_cb)
            estop_srv = node_test.create_service(Trigger, '/drone/emergency_stop', world_drone_emergency_stop_cb)
            print(f"[INFO] Drone services created:")
            print(f"  - /drone/arm: {arm_srv is not None}")
            print(f"  - /drone/takeoff: {takeoff_srv is not None}")
            print(f"  - /drone/land: {land_srv is not None}")
            print(f"  - /drone/emergency_stop: {estop_srv is not None}")
        print("[INFO] Drone control interface ready on /drone namespace")
        except Exception as e:
            print(f"[ERROR] Failed to create drone ROS2 interface: {e}")
            import traceback
            traceback.print_exc()
    
    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node_test,), daemon=True)
    thread.start()
    
    # Return node for cleanup
    return node_test
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


class TeeLogger:
    """Writes to both console and file"""
    def __init__(self, filename):
        self.terminal = sys.stdout
        self.log = open(filename, 'w', buffering=1)  # Line buffered
        
    def write(self, message):
        self.terminal.write(message)
        self.log.write(message)
        
    def flush(self):
        self.terminal.flush()
        self.log.flush()


def run_sim():
    # Setup logging to file
    log_dir = os.path.join(os.path.expanduser("~"), "ResQoUnity", "logs", "simulation")
    os.makedirs(log_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = os.path.join(log_dir, f"drone_sim_{timestamp}.log")
    
    # Redirect stdout to both console and file
    sys.stdout = TeeLogger(log_file)
    print(f"{'='*80}")
    print(f"SIMULATION LOG - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"Log file: {log_file}")
    print(f"{'='*80}\n")
    
    import omni.appwindow
    
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
                kp_pos=0.3,      # Position PID gains - REDUCED to prevent aggressive movement
                ki_pos=0.0,      # Disabled integral (can cause windup)
                kd_pos=0.2,      # Damping
                kp_alt=0.8,      # Altitude PID gains - REDUCED
                ki_alt=0.0,      # Disabled integral
                kd_alt=0.4,      # Damping
                kp_att=1.0,      # Attitude PID gains - REDUCED
                ki_att=0.0,
                kd_att=0.3,
                kp_yaw=0.5,      # Yaw rate PID gains - REDUCED
                ki_yaw=0.0,
                kd_yaw=0.1,
                max_vel=1.0,     # REDUCED max velocity
                max_climb_rate=0.8,  # REDUCED max climb rate
                max_angle_rate=1.0,    # REDUCED max roll/pitch rate (rad/s)
                max_yaw_rate=0.5,      # REDUCED max yaw rate (rad/s)
                hover_thrust=0.45      # Slightly increased from 0.35
            )
            # Auto-arm environment drones for immediate flight
            custom_rl_env.drone_controllers[str(i)].armed = True
            custom_rl_env.drone_armed[str(i)] = True
            # NOTE: Set mode to IDLE first, will switch to LOITER after first update() captures real position
            from drone_controller import DroneState
            custom_rl_env.drone_controllers[str(i)].set_mode(DroneState.IDLE)
            custom_rl_env.drone_mode[str(i)] = 'IDLE'  # Track mode in custom_rl_env
            print(f"[INFO] Drone {i} controller initialized and ARMED in IDLE (will switch to LOITER after spawn)")
    
    # reset environment
    print("[INFO] Resetting environment and getting initial observations...")
    obs, _ = env.get_observations()
    if not isinstance(obs, dict):
        obs = {"policy": obs}
    print("[INFO] Environment reset complete, starting simulation loop...")

    # Spawn world-level drone (separate from env system)
    # Use Isaac Core USD API for standalone objects (not Isaac Lab Articulation)
    world_drone_path = None
    world_drone_view = None
    world_drone_root_body_path = None  # Cached path to root body for force application
    # RigidPrim removed - not compatible with GPU physics mode
    world_drone_initialized = False
    enable_world_drone = False
    
    if args_cli.robot == "go2":  # Only spawn drone when using Go2 robots
        print("[INFO] Spawning world-level drone at /World/Drone...")
        try:
            import omni.isaac.core.utils.prims as prim_utils
            from pxr import UsdGeom, Gf, UsdPhysics
            
            # Get USD path from config
            drone_usd_path = QUADCOPTER_CFG.spawn.usd_path
            
            # CRITICAL FIX: Spawn inside environment scene so env.step() updates it
            # Changed from /World/Drone to /World/envs/env_0/Drone
            world_drone_path = "/World/envs/env_0/Drone"
            
            print(f"[INFO] Spawning drone at {world_drone_path} (inside env for physics stepping)")
            
            # Spawn drone USD directly to stage (Isaac Core API, not Orbit)
            prim_utils.create_prim(
                prim_path=world_drone_path,
                usd_path=drone_usd_path,
                translation=(0.0, 0.0, 2.5),  # 2.5m hover height
                scale=(5.0, 5.0, 5.0)  # 5x scale for visibility
            )
            
            world_drone_prim = prim_utils.get_prim_at_path(world_drone_path)
            
            if world_drone_prim and world_drone_prim.IsValid():
                print(f"[INFO] Drone USD loaded at {world_drone_path}")
                
                # Set physics properties via USD
                articulation_api = UsdPhysics.ArticulationRootAPI.Apply(world_drone_prim)
                print("[INFO] Drone spawned at position (0.0, 0.0, 2.5)")
                print("[INFO] ArticulationView will be initialized after first physics step")
            else:
                print("[ERROR] Failed to create drone prim")
                world_drone_path = None
            
            # Initialize drone controller (independent of physics view)
            if world_drone_path:
                from drone_controller import DroneController
                from drone_debug_logger import DroneDebugLogger
                
                custom_rl_env.world_drone_controller = DroneController(
                    robot_id="drone",
                    # FORCE-BASED CONTROL: Higher gains needed since forces must accelerate mass
                    kp_pos=1.2,      # Increased from 0.3 - position control
                    ki_pos=0.05,     # Added integral term - eliminates steady-state error
                    kd_pos=0.3,      # Increased damping
                    kp_alt=2.0,      # Increased from 0.8 - altitude is critical
                    ki_alt=0.15,     # Added integral - maintains hover
                    kd_alt=0.6,      # Increased damping - prevents oscillation
                    kp_att=1.0,      # Attitude gains (not used in simplified force control)
                    ki_att=0.0,
                    kd_att=0.3,
                    kp_yaw=0.5,
                    ki_yaw=0.0,
                    kd_yaw=0.1,
                    max_vel=2.0,     # Increased from 1.0 - forces can safely handle higher speeds
                    max_climb_rate=1.5,  # Increased from 0.8 - better takeoff
                    max_angle_rate=1.0,
                    max_yaw_rate=0.8,    # Increased yaw rate
                    hover_thrust=0.45    # Not used in force-based control
                )
                
                # AUTO-ARM: Start armed in IDLE, will switch to LOITER after first update
                from drone_controller import DroneState
                custom_rl_env.world_drone_controller.armed = True
                custom_rl_env.world_drone_controller.set_mode(DroneState.IDLE)
                
                # Initialize debug logger
                custom_rl_env.world_drone_logger = DroneDebugLogger("world_drone")
                print("[INFO] Drone controller initialized in ARMED mode (auto-armed)")
                print("[INFO] Mode set to IDLE - will switch to LOITER after spawn")
                print("[INFO] Debug logger enabled - verbose output active")
                
                # Add bottom-facing camera to drone
                try:
                    from omni.isaac.orbit.sensors import CameraCfg, Camera
                    drone_cam_cfg = CameraCfg(
                        prim_path="/World/Drone/base/bottom_cam",
                        update_period=0.1,
                        height=480,
                        width=640,
                        data_types=["rgb"],
                        spawn=sim_utils.PinholeCameraCfg(
                            focal_length=24.0,
                            focus_distance=400.0,
                            horizontal_aperture=20.955,
                            clipping_range=(0.1, 1.0e5)
                        ),
                        offset=CameraCfg.OffsetCfg(
                            pos=(0.0, 0.0, -0.1),  # Bottom mount
                            rot=(0.0, 0.707, 0.0, 0.707),  # 90° down (pitch down)
                            convention="ros"
                        ),
                    )
                    Camera(drone_cam_cfg)
                    print("[INFO] Drone bottom camera initialized")
                except Exception as cam_e:
                    print(f"[WARN] Failed to add drone camera: {cam_e}")
                
                enable_world_drone = True
        except Exception as e:
            print(f"[WARN] Failed to spawn world drone: {e}")
            import traceback
            traceback.print_exc()
            world_drone_path = None

    # initialize ROS2 node
    rclpy.init()
    base_node = RobotBaseNode(env_cfg.scene.num_envs, enable_world_drone=enable_world_drone)
    control_node = add_cmd_sub(env_cfg.scene.num_envs, enable_world_drone=enable_world_drone)

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
            
            # FORCE-BASED VELOCITY CONTROL for drones (bypass motor actions)
            # Motor commands don't work because USD has no thrust physics
            # Instead: calculate forces → integrate velocity → apply directly
            if args_cli.robot == "drone" or args_cli.robot == "quadcopter":
                # Access the robot articulation
                robot = env.unwrapped.scene["robot"]
                num_envs = env.unwrapped.num_envs
                
                # Calculate drone mass once  
                if not hasattr(custom_rl_env, 'env_drone_mass'):
                    custom_rl_env.env_drone_mass = 0.5  # kg (Crazyflie 5x scaled estimate)
                    print(f"[INFO] Environment drone mass: {custom_rl_env.env_drone_mass}kg")
                
                # Build velocity commands for all environments
                new_velocities_list = []
                
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
                        
                        # CRITICAL: Switch to LOITER after first update (now has real position)
                        # Wait 2 frames to ensure physics has stepped and position is valid
                        if not hasattr(custom_rl_env, '_loiter_switch_frame'):
                            custom_rl_env._loiter_switch_frame = {}
                        
                        if str(env_idx) not in custom_rl_env._loiter_switch_frame:
                            custom_rl_env._loiter_switch_frame[str(env_idx)] = 0
                        
                        custom_rl_env._loiter_switch_frame[str(env_idx)] += 1
                        
                        # DEBUG: Log the check on first few frames
                        if custom_rl_env._loiter_switch_frame[str(env_idx)] <= 5:
                            print(f"[DBG] Frame {custom_rl_env._loiter_switch_frame[str(env_idx)]} - Mode: {controller.mode.value}, Pos: ({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f})")
                        
                        # Switch to LOITER after 2 frames (ensure physics is initialized)
                        if (controller.mode.value == 'IDLE' and 
                            custom_rl_env.drone_mode[str(env_idx)] == 'IDLE' and
                            custom_rl_env._loiter_switch_frame[str(env_idx)] >= 2):
                            from drone_controller import DroneState
                            controller.set_mode(DroneState.LOITER)
                            custom_rl_env.drone_mode[str(env_idx)] = 'LOITER'
                            print(f"[INFO] Drone {env_idx} switched to LOITER at position ({current_pos[0]:.2f}, {current_pos[1]:.2f}, {current_pos[2]:.2f})")
                        
                        # FORCE-BASED CONTROL: Calculate desired velocities from PID
                        desired_vx = 0.0
                        desired_vy = 0.0
                        desired_vz = 0.0
                        
                        if controller.mode.value == 'POSITION' or controller.mode.value == 'LOITER':
                            # Position control: PID computes velocity from position error
                            error = controller.target_position - current_pos
                            desired_vx = controller.pid_x.update(error[0], dt)
                            desired_vy = controller.pid_y.update(error[1], dt)
                            desired_vz = controller.pid_z.update(error[2], dt)
                        
                        elif controller.mode.value == 'ALTITUDE_HOLD':
                            # Altitude control only
                            error_z = controller.target_altitude - current_pos[2]
                            desired_vz = controller.pid_z.update(error_z, dt)
                            cmd = custom_rl_env.base_command.get(str(env_idx), [0, 0, 0])
                            desired_vx = cmd[0]
                            desired_vy = cmd[1]
                        
                        elif controller.mode.value == 'VELOCITY':
                            # Direct velocity control
                            cmd = custom_rl_env.base_command.get(str(env_idx), [0, 0, 0])
                            if env_idx == 0:
                                altitude_cmd = getattr(custom_rl_env, 'drone_altitude', 0.0)
                            elif env_idx == 1:
                                altitude_cmd = getattr(custom_rl_env, 'drone_altitude_1', 0.0)
                            else:
                                altitude_cmd = 0.0
                            desired_vx = cmd[0]
                            desired_vy = cmd[1]
                            desired_vz = altitude_cmd
                        
                        elif controller.mode.value == 'LANDING':
                            desired_vz = controller.landing_velocity
                        
                        elif controller.mode.value == 'IDLE':
                            # IDLE: hover in place (zero velocity change)
                            desired_vx = 0.0
                            desired_vy = 0.0
                            desired_vz = 0.0
                        
                        # Clamp desired velocities
                        desired_vx = np.clip(desired_vx, -controller.max_vel, controller.max_vel)
                        desired_vy = np.clip(desired_vy, -controller.max_vel, controller.max_vel)
                        desired_vz = np.clip(desired_vz, -controller.max_climb_rate, controller.max_climb_rate)
                        
                        # Calculate forces needed (with gravity compensation)
                        desired_velocity_vec = np.array([desired_vx, desired_vy, desired_vz])
                        forces = calculate_drone_forces(
                            desired_velocity=desired_velocity_vec,
                            current_velocity=current_vel,
                            mass=custom_rl_env.env_drone_mass,
                            dt=dt,
                            gravity=9.81
                        )
                        
                        # Convert forces to velocity change: dv = (F/m) * dt
                        accel = forces / custom_rl_env.env_drone_mass
                        velocity_change = accel * dt
                        new_vel = current_vel + velocity_change
                        
                        # Apply damping (air resistance)
                        new_vel *= 0.95
                        
                        # Clamp for safety
                        new_vel = np.clip(new_vel, -5.0, 5.0)
                        
                        new_velocities_list.append(new_vel)
                        
                        # Log occasionally
                        if not hasattr(custom_rl_env, 'log_counter'):
                            custom_rl_env.log_counter = 0
                        if custom_rl_env.log_counter % 10 == 0:
                            print(f"[FORCE] env{env_idx} pos:({current_pos[0]:.2f},{current_pos[1]:.2f},{current_pos[2]:.2f}) des_vel:({desired_vx:.2f},{desired_vy:.2f},{desired_vz:.2f}) new_vel:({new_vel[0]:.2f},{new_vel[1]:.2f},{new_vel[2]:.2f})")
                        custom_rl_env.log_counter += 1
                    
                    else:
                        # Not armed: zero velocity
                        new_velocities_list.append(np.array([0.0, 0.0, 0.0]))
                
                # Convert to tensor and write to simulation
                velocities_tensor = torch.tensor(new_velocities_list, device=robot.device, dtype=torch.float32)
                
                # Add angular velocities (zero for now - could add yaw control later)
                angular_vels = torch.zeros((num_envs, 3), device=robot.device, dtype=torch.float32)
                full_vels = torch.cat([velocities_tensor, angular_vels], dim=1)  # Shape: (num_envs, 6)
                
                # Write velocities directly to simulation (bypasses action manager!)
                robot.write_root_velocity_to_sim(full_vels)
                
                # Use zero actions since we're controlling via velocities directly
                actions = torch.zeros((num_envs, 4), device=robot.device, dtype=torch.float32)
            
            # env stepping (MUST happen first to step physics)
            obs, _, _, _ = env.step(actions)
            
            # Publish odometry for environment robots (for drones: robot0, robot1, etc.)
            if args_cli.robot == "drone" or args_cli.robot == "quadcopter":
                robot = env.unwrapped.scene["robot"]
                num_envs = env.unwrapped.num_envs
                
                for env_idx in range(num_envs):
                    # Get current pose from physics
                    current_pos = robot.data.root_pos_w[env_idx]  # tensor [x, y, z]
                    current_quat = robot.data.root_quat_w[env_idx]  # tensor [w, x, y, z]
                    
                    # Publish odometry
                    try:
                        base_node.publish_odom(current_pos, current_quat, env_idx)
                        
                        # Debug: Log occasionally (every 60 frames = 1 second at 60 FPS)
                        if not hasattr(custom_rl_env, '_odom_dbg_ctr'):
                            custom_rl_env._odom_dbg_ctr = 0
                            print(f"[INFO] Starting odometry publishing for {num_envs} environment drone(s)")
                        
                        custom_rl_env._odom_dbg_ctr += 1
                        if custom_rl_env._odom_dbg_ctr % 60 == 0:
                            pos = current_pos.cpu().numpy()
                            print(f"[DBG ODOM] Published /robot{env_idx}/odom: pos=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
                    except Exception as e:
                        print(f"[ERROR] Failed to publish odometry for robot{env_idx}: {e}")
                        import traceback
                        traceback.print_exc()
            
            # World-level drone control (AFTER env.step to read latest physics state)
            if world_drone_path is not None and custom_rl_env.world_drone_controller is not None:
                # Lazy initialization: Create ArticulationView after first physics step
                if not world_drone_initialized:
                    try:
                        from omni.isaac.core.articulations import ArticulationView
                        print("[INFO] Initializing ArticulationView after physics step...")
                        world_drone_view = ArticulationView(
                            prim_paths_expr=world_drone_path,
                            name="world_drone_view"
                        )
                        world_drone_view.initialize()
                        
                        # NEW: Register with simulation context for proper rendering
                        try:
                            # Access simulation context through environment
                            sim = env.unwrapped.sim if hasattr(env.unwrapped, 'sim') else env.sim
                            
                            # Try to add to scene if supported
                            if hasattr(env, 'scene') and hasattr(env.scene, 'articulations'):
                                env.scene.articulations['world_drone'] = world_drone_view
                                print("[INFO] Drone registered with scene articulations")
                        except Exception as e:
                            print(f"[WARN] Could not register drone with scene: {e}")
                            print("[INFO] Will use manual render calls instead")
                        
                        # Use fixed mass estimate (get_masses() not available in all Isaac Sim versions)
                        # Crazyflie 2.x is about 27g, scaled 5x = ~0.675kg
                        # Use conservative estimate: 0.5 kg
                        custom_rl_env.world_drone_mass = 0.5
                        print(f"[INFO] Drone mass set to: {custom_rl_env.world_drone_mass} kg (Crazyflie scaled estimate)")
                        print(f"[INFO] Using Hybrid Physics-Velocity Control (GPU PhysX compatible)")
                        print(f"[INFO] Forces → Acceleration → Velocity integration (F=ma)")
                        
                        # CRITICAL: Initialize controller position immediately
                        positions, orientations = world_drone_view.get_world_poses()
                        velocities = world_drone_view.get_velocities()
                        
                        initial_pos = positions[0].cpu().numpy()
                        initial_quat = orientations[0].cpu().numpy()
                        initial_vel = velocities[0, :3].cpu().numpy()
                        
                        # Update controller with actual drone position
                        with custom_rl_env.world_drone_lock:
                            controller = custom_rl_env.world_drone_controller
                            logger = custom_rl_env.world_drone_logger
                            controller.current_position = initial_pos
                            controller.current_orientation = initial_quat
                            controller.current_euler = controller._quat_to_euler(initial_quat)
                            controller.current_velocity = initial_vel
                            
                            # Set target position to spawn position (for LOITER mode)
                            controller.target_position = initial_pos.copy()
                            print(f"[INFO] LOITER target set to spawn position: ({initial_pos[0]:.2f}, {initial_pos[1]:.2f}, {initial_pos[2]:.2f})")
                            
                            # Safety logging: Verify initialization is correct
                            print(f"[SAFETY] World drone initialized:")
                            print(f"  Current pos: ({initial_pos[0]:.3f}, {initial_pos[1]:.3f}, {initial_pos[2]:.3f})")
                            print(f"  Target pos:  ({controller.target_position[0]:.3f}, {controller.target_position[1]:.3f}, {controller.target_position[2]:.3f})")
                            print(f"  Mode: {controller.mode.value}, Armed: {controller.armed}")
                            
                            # Log initial state
                            logger.log_initialization(initial_pos, initial_vel, controller.mode.value)
                            logger.log_arm_event(True)  # Log that we're starting armed
                        
                        world_drone_initialized = True
                        print(f"[INFO] ArticulationView ready - found {world_drone_view.count} drone(s)")
                        
                        # CRITICAL DEBUG: What did we actually find?
                        print(f"[DEBUG] ArticulationView prim paths: {world_drone_view.prim_paths}")
                        print(f"[DEBUG] Controller initialized at position: ({initial_pos[0]:.2f}, {initial_pos[1]:.2f}, {initial_pos[2]:.2f})")
                        print(f"[DEBUG] Drone has {world_drone_view.num_dof} DOFs: {world_drone_view.dof_names}")
                        
                        # Find and cache the root body prim for force application
                        import omni.isaac.core.utils.prims as prim_utils
                        drone_prim = prim_utils.get_prim_at_path(world_drone_path)
                        print(f"[DEBUG] Prim at {world_drone_path} exists: {drone_prim.IsValid()}")
                        
                        # Cache the root body prim path for efficient force application
                        # Find "base" link or first rigid body
                        stage = drone_prim.GetStage()
                        for child in drone_prim.GetChildren():
                            if child.GetTypeName() == "Xform" and "base" in child.GetName().lower():
                                world_drone_root_body_path = str(child.GetPath())
                                print(f"[DEBUG] Found root body at: {world_drone_root_body_path}")
                                break
                        if not world_drone_root_body_path:
                            from pxr import UsdPhysics
                            for child in drone_prim.GetChildren():
                                if UsdPhysics.RigidBodyAPI(child):
                                    world_drone_root_body_path = str(child.GetPath())
                                    print(f"[DEBUG] Found root body (fallback) at: {world_drone_root_body_path}")
                                    break
                        
                        # NOTE: RigidPrim removed - causes GPU physics errors
                        # Force application now uses PhysxSchema.PhysxRigidBodyAPI directly
                        
                        # Check for articulation root and joint drive settings
                        from pxr import UsdPhysics, PhysxSchema
                        if drone_prim.IsValid():
                            # Walk the hierarchy to find the actual articulation root
                            stage = drone_prim.GetStage()
                            for prim in stage.Traverse():
                                if prim.GetPath().pathString.startswith(world_drone_path):
                                    if UsdPhysics.ArticulationRootAPI(prim):
                                        print(f"[DEBUG] Found articulation root at: {prim.GetPath()}")
                                    
                                    # Check joint drive configuration
                                    joint = UsdPhysics.RevoluteJoint(prim)
                                    if joint:
                                        joint_name = prim.GetName()
                                        drive_api = UsdPhysics.DriveAPI(prim, "angular")
                                        if drive_api:
                                            max_force = drive_api.GetMaxForceAttr().Get()
                                            damping = drive_api.GetDampingAttr().Get()
                                            stiffness = drive_api.GetStiffnessAttr().Get()
                                            print(f"[DEBUG] Joint {joint_name}: maxForce={max_force}, damping={damping}, stiffness={stiffness}")
                    except Exception as e:
                        print(f"[WARN] Drone view initialization failed (will retry next frame): {e}")
                        # Don't set initialized=True, will retry next frame
                
                # Only proceed if view is initialized
                if world_drone_initialized and world_drone_view is not None:
                    try:
                        dt = env.unwrapped.step_dt if hasattr(env.unwrapped, 'step_dt') else (1.0 / 60.0)
                        
                        # Get current state using ArticulationView API (GPU-safe)
                        positions, orientations = world_drone_view.get_world_poses()  # Returns (N, 3), (N, 4)
                        velocities = world_drone_view.get_velocities()  # Returns (N, 6) [linear, angular]
                        joint_positions = world_drone_view.get_joint_positions()  # For publishing
                        
                        # Convert to numpy (index 0 since we have 1 drone)
                        current_pos = positions[0].cpu().numpy()  # [x, y, z]
                        current_quat = orientations[0].cpu().numpy()  # [w, x, y, z]
                        current_vel = velocities[0, :3].cpu().numpy()  # First 3 are linear velocity
                        
                        # NOTE: Cache is updated AFTER control loop (see below) so ROS2 gets current position
                        
                        # THREAD-SAFE: Lock while reading controller state and commands
                        with custom_rl_env.world_drone_lock:
                            controller = custom_rl_env.world_drone_controller
                            logger = custom_rl_env.world_drone_logger
                            
                            # ALWAYS update controller state (even when disarmed)
                            # This ensures position is initialized before first takeoff
                            controller.current_orientation = current_quat
                            controller.current_euler = controller._quat_to_euler(current_quat)
                            controller.current_position = current_pos
                            controller.current_velocity = current_vel
                            
                            if controller.armed:
                                # CRITICAL: Switch to LOITER after first update (now has real position)
                                if controller.mode.value == 'IDLE':
                                    from drone_controller import DroneState
                                    controller.set_mode(DroneState.LOITER)
                                    print(f"[INFO] World drone switched to LOITER at position ({current_pos[0]:.2f}, {current_pos[1]:.2f}, {current_pos[2]:.2f})")
                                
                                # Calculate position error for logging
                                pos_error = controller.target_position - current_pos
                                # Update controller PID loops
                                controller.update(dt, current_pos, current_vel)
                                
                                # Get motor commands from controller
                                motor_cmds = controller.compute_motor_command()
                                
                                # For VELOCITY mode, convert velocity inputs to motor commands
                                if controller.mode.value == 'VELOCITY':
                                    # Get velocity commands from cmd_vel/keyboard (thread-safe under lock)
                                    vx, vy, yaw_rate = custom_rl_env.world_drone_command[:]  # Copy list
                                    altitude_cmd = custom_rl_env.world_drone_altitude
                                    
                                    # Convert velocities to motor commands
                                    desired_pitch = -vx * 0.3
                                    desired_roll = vy * 0.3
                                    thrust = controller.hover_thrust + altitude_cmd * 0.2
                                    thrust = np.clip(thrust, 0.0, 1.0)
                                    
                                    # Motor mixing
                                    motor_cmds = controller._motor_mixer(thrust, desired_roll, desired_pitch, yaw_rate * 0.1)
                                
                                # Apply motor commands - HYBRID APPROACH:
                                # DISABLED: Joint velocity targets lock the drone in place
                                # The force-based control below handles all movement
                                
                                # max_motor_velocity = 2000.0  # rad/s (~19000 RPM)
                                # motor_velocities = motor_cmds * max_motor_velocity
                                
                                # Visual: Spin the rotors
                                # velocity_targets = torch.from_numpy(np.array([motor_velocities], dtype=np.float32))
                                # world_drone_view.set_joint_velocity_targets(velocity_targets)
                                
                                # FORCE-BASED CONTROL (from OmniDrones approach)
                                # Calculate desired velocities from PID controllers based on flight mode
                                
                                desired_vx = 0.0
                                desired_vy = 0.0
                                desired_vz = 0.0
                                desired_yaw_rate = 0.0
                                
                                if controller.mode.value == 'POSITION' or controller.mode.value == 'LOITER':
                                    # Position control: PID computes velocity from position error
                                    error = controller.target_position - current_pos
                                    desired_vx = controller.pid_x.update(error[0], dt)
                                    desired_vy = controller.pid_y.update(error[1], dt)
                                    desired_vz = controller.pid_z.update(error[2], dt)
                                
                                elif controller.mode.value == 'ALTITUDE_HOLD':
                                    # Altitude control only, XY from cmd_vel
                                    error_z = controller.target_altitude - current_pos[2]
                                    desired_vz = controller.pid_z.update(error_z, dt)
                                    with custom_rl_env.world_drone_lock:
                                        cmd_vel = custom_rl_env.world_drone_command
                                        desired_vx = cmd_vel[0]
                                        desired_vy = cmd_vel[1]
                                        desired_yaw_rate = cmd_vel[2]
                                
                                elif controller.mode.value == 'LANDING':
                                    desired_vz = controller.landing_velocity  # Fixed descent rate
                                
                                elif controller.mode.value == 'VELOCITY':
                                    # Direct velocity control from cmd_vel
                                    with custom_rl_env.world_drone_lock:
                                        cmd_vel = custom_rl_env.world_drone_command
                                        desired_vx = cmd_vel[0]
                                        desired_vy = cmd_vel[1]
                                        desired_yaw_rate = cmd_vel[2]
                                        desired_vz = custom_rl_env.world_drone_altitude
                                
                                # Convert desired velocities to forces (F=ma approach)
                                
                                # Get device from existing tensor (positions is already on GPU)
                                device = positions.device if hasattr(positions, 'device') else 'cuda:0'
                                
                                # Clamp velocities to safe limits
                                desired_vx = np.clip(desired_vx, -controller.max_vel, controller.max_vel)
                                desired_vy = np.clip(desired_vy, -controller.max_vel, controller.max_vel)
                                desired_vz = np.clip(desired_vz, -controller.max_climb_rate, controller.max_climb_rate)
                                desired_yaw_rate = np.clip(desired_yaw_rate, -controller.max_yaw_rate, controller.max_yaw_rate)
                                
                                # Get drone mass from cached value
                                drone_mass = custom_rl_env.world_drone_mass
                                if drone_mass is None:
                                    print("[ERROR] Drone mass not calculated! Using fallback.")
                                    drone_mass = 0.5
                                
                                # Calculate forces needed (with gravity compensation)
                                desired_velocity_vec = np.array([desired_vx, desired_vy, desired_vz])
                                forces = calculate_drone_forces(
                                    desired_velocity=desired_velocity_vec,
                                    current_velocity=current_vel,
                                    mass=drone_mass,
                                    dt=1.0/60.0,
                                    gravity=9.81
                                )
                                
                                # Convert to torch tensor for ArticulationView
                                # Forces are in world frame, shape: (num_drones, 3)
                                force_tensor = torch.tensor(
                                    [[forces[0], forces[1], forces[2]]],
                                    dtype=torch.float32,
                                    device=device
                                )
                                
                                # Torque for yaw control (simplified, just yaw axis)
                                # Calculate torque needed for yaw rate: τ = I * α
                                # Moment of inertia (rough estimate for scaled Crazyflie)
                                yaw_inertia = 0.01  # kg*m^2 (tunable)
                                yaw_error = desired_yaw_rate - 0.0  # Assume current yaw rate is ~0 for simplicity
                                yaw_torque = yaw_inertia * yaw_error * 10.0  # Gain = 10
                                
                                torque_tensor = torch.tensor(
                                    [[0.0, 0.0, yaw_torque]],  # Only yaw torque
                                    dtype=torch.float32,
                                    device=device
                                )
                                
                                # HYBRID PHYSICS-VELOCITY CONTROL
                                # GPU PhysX doesn't allow direct force application via USD attributes
                                # So we use physics to CALCULATE what the velocity should be, then apply it
                                # This gives us physics-based behavior within GPU constraints
                                
                                try:
                                    # Convert forces to acceleration: a = F/m
                                    acceleration = forces / drone_mass  # (3,) array in m/s²
                                    
                                    # Get current velocity from physics engine
                                    current_vels = world_drone_view.get_velocities()  # (1, 6) tensor [vx, vy, vz, wx, wy, wz]
                                    current_linear_vel = current_vels[0, :3].cpu().numpy()  # (3,) array
                                    current_angular_vel = current_vels[0, 3:6].cpu().numpy()  # (3,) array
                                    
                                    # Integrate: v_new = v_current + a * dt
                                    dt = 1.0 / 60.0  # 60 Hz physics
                                    new_linear_vel = current_linear_vel + acceleration * dt
                                    
                                    # REMOVED: Manual damping was double-damping with USD linear_damping config
                                    # The USD already has linear_damping=0.05, so we don't need to apply it again
                                    # damping = 0.95
                                    # new_linear_vel = new_linear_vel * damping
                                    
                                    # Clamp velocities for safety
                                    max_velocity = 5.0  # m/s
                                    new_linear_vel = np.clip(new_linear_vel, -max_velocity, max_velocity)
                                    
                                    # Angular velocity for yaw control
                                    # Simple proportional control for yaw angular velocity
                                    yaw_accel = yaw_torque / (drone_mass * 0.1)  # Assume small moment of inertia
                                    new_angular_vel = current_angular_vel.copy()
                                    new_angular_vel[2] = current_angular_vel[2] + yaw_accel * dt
                                    new_angular_vel[2] = np.clip(new_angular_vel[2], -2.0, 2.0)  # Max 2 rad/s yaw rate
                                    
                                    # Apply the calculated velocities
                                    new_vels = torch.tensor([
                                        [new_linear_vel[0], new_linear_vel[1], new_linear_vel[2],
                                         new_angular_vel[0], new_angular_vel[1], new_angular_vel[2]]
                                    ], dtype=torch.float32, device=device)
                                    
                                    world_drone_view.set_velocities(new_vels)
                                    
                                    # CRITICAL: Force viewport update for GPU PhysX
                                    # World-level objects need explicit render flush
                                    try:
                                        sim = env.unwrapped.sim if hasattr(env.unwrapped, 'sim') else env.sim
                                        sim.render()  # Sync GPU physics state to viewport
                                        
                                        # Verify render was called (log occasionally)
                                        if hasattr(logger, 'frame_count') and logger.frame_count % 20 == 0:
                                            print(f"[DBG RENDER] Viewport sync triggered at pos ({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f})")
                                    except AttributeError:
                                        # Fallback: try direct simulation app update
                                        try:
                                            from omni.isaac.kit import SimulationApp
                                            app = SimulationApp.instance()
                                            if app:
                                                app.update()
                                        except:
                                            pass  # Last resort: hope env.step() next frame does it
                                    
                                    # GPU PhysX viewport sync: Render flush added above
                                    # set_velocities() updates internal state, sim.render() syncs viewport
                                    # This is the documented approach for GPU PhysX transform updates
                                    
                                    # For logging - track the force we calculated
                                    applied_force = forces  
                                    
                                except Exception as physics_error:
                                    print(f"[ERROR] Hybrid physics-velocity control failed: {physics_error}")
                                    import traceback
                                    traceback.print_exc()
                                    applied_force = np.zeros(3)
                                
                                # For logging - get what actually happened
                                vels_after = world_drone_view.get_velocities()
                                applied_vel = vels_after[0, :3].cpu().numpy()
                                
                                # Detailed frame logging with forces
                                logger.log_frame(
                                    current_pos=current_pos,
                                    current_vel=current_vel,
                                    target_pos=controller.target_position,
                                    mode=controller.mode.value,
                                    desired_vel=np.array([desired_vx, desired_vy, desired_vz]),
                                    error=pos_error,
                                    armed=controller.armed,
                                    actual_vel_applied=applied_vel,
                                    applied_force=applied_force
                                )
                                
                                # ALSO spin the rotors for visual effect
                                # DISABLED: Joint velocity targets were locking the drone in place
                                # max_motor_velocity = 2000.0  # rad/s
                                # motor_velocities = motor_cmds * max_motor_velocity
                                # velocity_targets = torch.from_numpy(np.array([motor_velocities], dtype=np.float32))
                                # world_drone_view.set_joint_velocity_targets(velocity_targets)
                                
                                # Old debug output replaced by comprehensive logger
                            else:
                                # Not armed: gradually reduce velocity (simulate gravity pulling down + ground friction)
                                try:
                                    current_vels = world_drone_view.get_velocities()
                                    # Apply strong damping when disarmed (drone settles quickly)
                                    damped_vels = current_vels.clone()
                                    damped_vels[0, :] *= 0.8  # 80% damping per frame
                                    world_drone_view.set_velocities(damped_vels)
                                except Exception:
                                    pass  # Best effort
                                
                                # Also zero motor velocities
                                zero_motor = torch.zeros((1, 4), dtype=torch.float32)
                                world_drone_view.set_joint_velocity_targets(zero_motor)
                                
                                applied_force = np.zeros(3)
                                applied_vel = np.zeros(3)
                        
                    except Exception as e:
                        print(f"[ERROR] Drone control failed: {e}")
                        import traceback
                        traceback.print_exc()
                        # Emergency: zero all velocities to prevent unsafe behavior
                        try:
                            if world_drone_view is not None:
                                current_vels = world_drone_view.get_velocities()
                                current_vels[0, :6] = 0.0  # Zero all root velocities
                                world_drone_view.set_velocities(current_vels)
                                zero_motor = torch.zeros((1, 4), dtype=torch.float32)
                                world_drone_view.set_joint_velocity_targets(zero_motor)
                        except:
                            pass  # Best effort
                
                # Update cache AFTER control is applied (so ROS2 gets current position)
                if world_drone_initialized and world_drone_view is not None:
                    try:
                        positions, orientations = world_drone_view.get_world_poses()
                        velocities = world_drone_view.get_velocities()
                        joint_positions = world_drone_view.get_joint_positions()
                        
                        # DEBUG: Verify cache is being updated
                        if not hasattr(custom_rl_env, '_dbg_ctr'):
                            custom_rl_env._dbg_ctr = 0
                        custom_rl_env._dbg_ctr += 1
                        if custom_rl_env._dbg_ctr % 60 == 0:
                            p = positions[0].cpu().numpy()
                            print(f"[DBG CACHE] Frame {custom_rl_env._dbg_ctr}: Updating cache with pos=({p[0]:.3f},{p[1]:.3f},{p[2]:.3f})")
                        
                        custom_rl_env.world_drone_state_cache = {
                            'position': positions[0],
                            'orientation': orientations[0],
                            'linear_velocity': velocities[0, :3],
                            'angular_velocity': velocities[0, 3:],
                            'joint_names': world_drone_view.dof_names,
                            'joint_positions': joint_positions[0]
                        }
                    except Exception as e:
                        print(f"[ERROR] Cache update exception: {e}")
            
            # Process observations
            if not isinstance(obs, dict):
                obs = {"policy": obs}
            pub_robo_data_ros2(args_cli.robot, env_cfg.scene.num_envs, base_node, env, annotator_lst, start_time)
            
            # Publish world drone data (if enabled and initialized)
            # Use cached state from control loop to avoid GPU PhysX restrictions
            if world_drone_initialized and enable_world_drone:
                if hasattr(custom_rl_env, 'world_drone_state_cache'):
                    try:
                        cache = custom_rl_env.world_drone_state_cache
                        
                        # DEBUG: Print what we're publishing
                        if hasattr(custom_rl_env, '_dbg_ctr') and custom_rl_env._dbg_ctr % 60 == 0:
                            p = cache['position'].cpu().numpy()
                            print(f"[DBG ROS2] Publishing to /drone/odom: pos=({p[0]:.3f},{p[1]:.3f},{p[2]:.3f})")
                    
                    # Publish odometry (position + orientation)
                    base_node.publish_drone_odom(
                            cache['position'],  # xyz position
                            cache['orientation']  # wxyz quaternion
                    )
                    
                    # Publish IMU (orientation + velocities)
                    base_node.publish_drone_imu(
                            cache['orientation'],  # wxyz quaternion
                            cache['linear_velocity'],  # linear velocity
                            cache['angular_velocity']  # angular velocity
                    )
                    
                    # Publish joint states
                        base_node.publish_drone_joints(
                            cache['joint_names'],
                            cache['joint_positions']
                        )
                except Exception as e:
                        # Log publishing errors for debugging
                        if not hasattr(base_node, '_publish_error_logged'):
                            print(f"[ERROR] Failed to publish world drone data: {e}")
                            import traceback
                            traceback.print_exc()
                            base_node._publish_error_logged = True  # Only log once to avoid spam
    
    # Cleanup on exit
    print("[INFO] Shutting down simulation...")
    env.close()
    
    # Clean up ROS2 resources
    try:
        control_node.destroy_node()
        base_node.destroy_node()
        rclpy.shutdown()
        print("[INFO] ROS2 shutdown complete")
    except Exception as e:
        print(f"[WARN] ROS2 cleanup error: {e}")
    
    print("[INFO] Simulation shutdown complete")