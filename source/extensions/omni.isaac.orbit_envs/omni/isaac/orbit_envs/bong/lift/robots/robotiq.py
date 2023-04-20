# Copyright (c) 2022, NVIDIA CORPORATION & AFFILIATES, ETH Zurich, and University of Toronto
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the Franka Emika robots.

The following configurations are available:

* :obj:`FRANKA_PANDA_ARM_WITH_PANDA_HAND_CFG`: Franka Emika Panda robot with Panda hand

Reference: https://github.com/frankaemika/franka_ros
"""


# from omni.isaac.orbit.actuators.config.franka import PANDA_HAND_MIMIC_GROUP_CFG
from omni.isaac.orbit.actuators.group import ActuatorGroupCfg, GripperActuatorGroupCfg
from omni.isaac.orbit.actuators.group.actuator_group_cfg import ActuatorControlCfg
from omni.isaac.orbit.actuators.model import ImplicitActuatorCfg
# from omni.isaac.orbit.utils.assets import ISAAC_ORBIT_NUCLEUS_DIR
from omni.isaac.orbit.robots.single_arm import SingleArmManipulatorCfg

# _FRANKA_PANDA_ARM_INSTANCEABLE_USD = f"{ISAAC_ORBIT_NUCLEUS_DIR}/Robots/FrankaEmika/panda_instanceable.usd"
# _ROBOTIQ_WRIST_INSTANCEABLE_USD = "/home/bong/.local/share/ov/pkg/isaac_sim-2022.2.1/Orbit/source/extensions/omni.isaac.orbit_envs/omni/isaac/orbit_envs/bong/robotiq_wrist.usd"
_ROBOTIQ_WRIST_INSTANCEABLE_USD = "/home/bong/.local/share/ov/pkg/isaac_sim-2022.2.1/Orbit/source/extensions/omni.isaac.orbit_envs/omni/isaac/orbit_envs/bong/robotiq_wrist_resized_direction.usd"
# _ROBOTIQ_WRIST_INSTANCEABLE_USD = "/home/bong/.local/share/ov/pkg/isaac_sim-2022.2.1/Orbit/source/extensions/omni.isaac.orbit_envs/omni/isaac/orbit_envs/bong/robotiq_wrist_resized_direction_mass.usd"

VELOCITY_LIMIT = 100
TORQUE_LIMIT = 50
STIFFNESS = 50
DAMPING = 7

ROBOTIQ_WRIST_WITH_ROBOTIQ_CFG = SingleArmManipulatorCfg(
    meta_info=SingleArmManipulatorCfg.MetaInfoCfg(
        usd_path=_ROBOTIQ_WRIST_INSTANCEABLE_USD,
        arm_num_dof=6,
        tool_num_dof=4,
        tool_sites_names=["bot_link_1", "bot_link_2", "top_link_1", "top_link_2"],  # xform
    ),
    init_state=SingleArmManipulatorCfg.InitialStateCfg(   # revjoint
        pos=[0.2, 0, 0.3],
        # pos=[0, 0, 0],
        # rot=[0.707, 0, 0.707, 0],
        rot=[0.5, 0.5, 0.5, 0.5],  # {action:global} = {x, z}, {y, x}, {z, y}
        dof_pos={
            "trans_x": 0.0,
            "trans_y": 0.0,
            "trans_z": 0.0,
            "rev_x": 0.0,
            "rev_y": 0.0,
            "rev_z": 0.0,
            "bot_joint_0_p": 0.0,
            "top_joint_0_p": 0.0,
            "bot_joint_1_p": 0.0,
            "top_joint_1_p": 0.0,
            "bot_joint_2_p": 0.0,
            "top_joint_2_p" : 0.0,
        },
        dof_vel={".*": 0.0},

    ),
    ee_info=SingleArmManipulatorCfg.EndEffectorFrameCfg(
        body_name="base", pos_offset=(0.0, 0.0, 0.12), rot_offset=(1.0, 0.0, 0.0, 0.0)  # xform / head
    ),
    rigid_props=SingleArmManipulatorCfg.RigidBodyPropertiesCfg(
        # solver_position_iteration_count=16,  # bong
        # solver_velocity_iteration_count=1,  # bong
        disable_gravity=True,
        max_depenetration_velocity=5.0,
    ),
    collision_props=SingleArmManipulatorCfg.CollisionPropertiesCfg(
        contact_offset=0.005,
        rest_offset=0.0,
    ),
    articulation_props=SingleArmManipulatorCfg.ArticulationRootPropertiesCfg(
        enable_self_collisions=True,
    ),
    physics_material=SingleArmManipulatorCfg.PhysicsMaterialCfg(
        static_friction=5, dynamic_friction=5, restitution=0.0, prim_path="/World/Materials/gripperMaterial"
    ),
    actuator_groups={
        "wrist_trans": ActuatorGroupCfg(
            dof_names=["trans_[x-z]"],
            model_cfg=ImplicitActuatorCfg(velocity_limit=VELOCITY_LIMIT, torque_limit=TORQUE_LIMIT),
            control_cfg=ActuatorControlCfg(
                command_types=["p_abs"],
                stiffness={".*": STIFFNESS},
                damping={".*": DAMPING},
                dof_pos_offset={
                    "trans_x": 0.0,
                    "trans_y": 0.0,
                    "trans_z": 0.0,
                },
            ),
        ),
        # "wrist_rev": ActuatorGroupCfg(
        #     dof_names=["rev_[x]"],
        #     model_cfg=ImplicitActuatorCfg(velocity_limit=VELOCITY_LIMIT, torque_limit=TORQUE_LIMIT),
        #     control_cfg=ActuatorControlCfg(
        #         command_types=["p_abs"],
        #         stiffness={".*": STIFFNESS},
        #         damping={".*": DAMPING},
        #         dof_pos_offset={"rev_x": 0, "rev_y": 0, "rev_z": 0.0},
        #     ),
        # ),
        "robotiq_hand": GripperActuatorGroupCfg(
            dof_names=["bot_joint_0_p", "top_joint_0_p", "bot_joint_1_p", "top_joint_1_p", "bot_joint_2_p", "top_joint_2_p"],
            model_cfg=ImplicitActuatorCfg(velocity_limit=VELOCITY_LIMIT, torque_limit=1000),
            control_cfg=ActuatorControlCfg(command_types=["p_abs"], stiffness={".*": 400}, damping={".*": 20}),
            mimic_multiplier={"bot_joint_0_p": 1, "top_joint_0_p": -1, "bot_joint_1_p": 1, "top_joint_1_p": -1, "bot_joint_2_p": -1, "top_joint_2_p": 1},
            # speed=1,
            open_dof_pos=0,
            # close_dof_pos=0.3,
            # close_dof_pos=0.785398,
            close_dof_pos=0.4,
        )
    },
)
"""Configuration of Franka arm with Franka Hand using implicit actuator models."""
