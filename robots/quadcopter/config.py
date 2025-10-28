"""Configuration for Quadcopter Drone compatible with Isaac Sim 2023.1.1"""

import omni.isaac.orbit.sim as sim_utils
from omni.isaac.orbit.actuators import ImplicitActuatorCfg
from omni.isaac.orbit.assets.articulation import ArticulationCfg


QUADCOPTER_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        # Using Isaac Sim's built-in Crazyflie quadcopter from Omniverse nucleus
        usd_path="http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/2023.1.1/Isaac/Robots/Crazyflie/cf2x.usd",
        scale=(5.0, 5.0, 5.0),  # Scale up 5x for better visibility (Crazyflie is tiny!)
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.05,
            angular_damping=0.05,
            max_linear_velocity=100.0,
            max_angular_velocity=100.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=4
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 1.0),  # Start at 1m height
        rot=(1.0, 0.0, 0.0, 0.0),  # Identity quaternion
        joint_pos={
            # Quadcopter typically has 4 rotor joints
            "m1_joint": 0.0,
            "m2_joint": 0.0,
            "m3_joint": 0.0,
            "m4_joint": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "rotors": ImplicitActuatorCfg(
            joint_names_expr=["m1_joint", "m2_joint", "m3_joint", "m4_joint"],
            effort_limit=1.0,
            velocity_limit=100.0,
            stiffness=0.0,
            damping=0.01,
        ),
    },
)


# Alternative: Simple rigid body drone (no articulation, simpler)
SIMPLE_DRONE_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        # Fallback: create a simple box-shaped drone if Crazyflie not available
        usd_path="http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/2023.1.1/Isaac/Props/Blocks/DexCube/dex_cube_instanceable.usd",
        activate_contact_sensors=False,
        scale=(0.3, 0.3, 0.1),  # Make it drone-shaped
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            linear_damping=0.1,
            angular_damping=0.1,
            max_linear_velocity=50.0,
            max_angular_velocity=50.0,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 1.5),  # Start at 1.5m height
        rot=(1.0, 0.0, 0.0, 0.0),
    ),
)

