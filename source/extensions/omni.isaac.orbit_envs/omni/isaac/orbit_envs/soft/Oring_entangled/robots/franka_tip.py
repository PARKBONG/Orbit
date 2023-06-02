# Copyright (c) 2022, NVIDIA CORPORATION & AFFILIATES, ETH Zurich, and University of Toronto
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause


from omni.isaac.orbit.actuators.group import ActuatorGroupCfg, GripperActuatorGroupCfg
from omni.isaac.orbit.actuators.group.actuator_group_cfg import ActuatorControlCfg
from omni.isaac.orbit.actuators.model import ImplicitActuatorCfg
# from omni.isaac.orbit.utils.assets import ISAAC_ORBIT_NUCLEUS_DIR
from omni.isaac.orbit.robots.single_arm import SingleArmManipulatorCfg

_FRANKA_INSTANCEABLE_USD = "/home/bong/.local/share/ov/pkg/isaac_sim-2022.2.1/Orbit/source/extensions/omni.isaac.orbit_envs/omni/isaac/orbit_envs/soft/Oring_entangled/usd/franka_gripper_fix.usd"  # open
#
VELOCITY_LIMIT = 1000
TORQUE_LIMIT = 1000
STIFFNESS = 2000
DAMPING = 100

FRANKA_TIP_CFG = SingleArmManipulatorCfg(
    meta_info=SingleArmManipulatorCfg.MetaInfoCfg(
        usd_path=_FRANKA_INSTANCEABLE_USD,
        arm_num_dof=6,
        tool_num_dof=0,
        # tool_sites_names=[],  # xform
    ),
    init_state=SingleArmManipulatorCfg.InitialStateCfg(   # revjoint
        pos=[0.0, 0.5, 1.0],
        # pos=[0.0, 0.5, 0.2],
        rot=[1, 0, 0, 0],  # {action:global} = {x, z}, {y, x}, {z, y}
        dof_pos={
            "joint_x": 0.0,
            "joint_y": 0.0,
            "joint_z": 0.0,
            "rev_x": 0.0,
            "rev_y": 0.0,
            "rev_z": 0.0,
        },
        dof_vel={".*": 0.0},

    ),
    ee_info=SingleArmManipulatorCfg.EndEffectorFrameCfg(
        body_name="panda_hand", pos_offset=(0.0, 0.0, 0.0), rot_offset=(1.0, 0.0, 0.0, 0.0)  # xform / head
    ),
    rigid_props=SingleArmManipulatorCfg.RigidBodyPropertiesCfg(
        # solver_position_iteration_count=16,  # bong
        # solver_velocity_iteration_count=1,  # bong
        disable_gravity=True,
        max_depenetration_velocity=5.0,
    ),
    collision_props=SingleArmManipulatorCfg.CollisionPropertiesCfg(
        # collision_enabled=True,
        contact_offset=0.005,
        rest_offset=0.0,
    ),
    articulation_props=SingleArmManipulatorCfg.ArticulationRootPropertiesCfg(
        enable_self_collisions=True,
    ),
    # physics_material=SingleArmManipulatorCfg.PhysicsMaterialCfg(
    #     static_friction=5, dynamic_friction=5, restitution=0.0, prim_path="/World/Materials/gripperMaterial"
    # ),
    actuator_groups={
        "wrist_trans": ActuatorGroupCfg(
            dof_names=["joint_[x-z]"],
            model_cfg=ImplicitActuatorCfg(velocity_limit=VELOCITY_LIMIT, torque_limit=TORQUE_LIMIT),
            control_cfg=ActuatorControlCfg(
                command_types=["p_abs"],
                stiffness={".*": STIFFNESS},
                damping={".*": DAMPING},
                dof_pos_offset={
                    "joint_x": 0.0,
                    "joint_y": 0.0,
                    "joint_z": 0.0,
                },
            ),
        ),
        "wrist_rev": ActuatorGroupCfg(
            dof_names=["rev_[x-z]"],
            model_cfg=ImplicitActuatorCfg(velocity_limit=VELOCITY_LIMIT, torque_limit=TORQUE_LIMIT),
            control_cfg=ActuatorControlCfg(
                command_types=["p_abs"],
                stiffness={".*": STIFFNESS},
                damping={".*": DAMPING},
                dof_pos_offset={
                    "rev_x": 0.0,
                    "rev_y": 0.0,
                    "rev_z": 0.0,
                },
            ),
        ),
    },
)
