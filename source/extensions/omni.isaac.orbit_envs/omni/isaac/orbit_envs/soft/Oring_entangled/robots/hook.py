# Copyright (c) 2022, NVIDIA CORPORATION & AFFILIATES, ETH Zurich, and University of Toronto
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause


from omni.isaac.orbit.actuators.group import ActuatorGroupCfg, GripperActuatorGroupCfg
from omni.isaac.orbit.actuators.group.actuator_group_cfg import ActuatorControlCfg
from omni.isaac.orbit.actuators.model import ImplicitActuatorCfg
# from omni.isaac.orbit.utils.assets import ISAAC_ORBIT_NUCLEUS_DIR
from omni.isaac.orbit.robots.single_arm import SingleArmManipulatorCfg

# _HOOK_INSTANCEABLE_USD = "/home/bong/.local/share/ov/pkg/isaac_sim-2022.2.1/Orbit/source/extensions/omni.isaac.orbit_envs/omni/isaac/orbit_envs/soft/Oring/usd/4DOF_HOOK_size_2.usd"  # small
# _HOOK_INSTANCEABLE_USD = "/home/bong/.local/share/ov/pkg/isaac_sim-2022.2.1/Orbit/source/extensions/omni.isaac.orbit_envs/omni/isaac/orbit_envs/soft/Oring/usd/4DOF_HOOK_large.usd"  # large
_HOOK_INSTANCEABLE_USD = "/home/bong/.local/share/ov/pkg/isaac_sim-2022.2.1/Orbit/source/extensions/omni.isaac.orbit_envs/omni/isaac/orbit_envs/soft/Oring/usd/4DOF_HOOK_large_open_final3.usd"  # open

VELOCITY_LIMIT = 1000000
TORQUE_LIMIT = 1000000
STIFFNESS = 20000
DAMPING = 1000

HOOK_CFG = SingleArmManipulatorCfg(
    meta_info=SingleArmManipulatorCfg.MetaInfoCfg(
        usd_path=_HOOK_INSTANCEABLE_USD,
        arm_num_dof=4,
        tool_num_dof=2,
        # tool_sites_names=[],  # xform
    ),
    init_state=SingleArmManipulatorCfg.InitialStateCfg(   # revjoint
        pos=[0.0, 0.0, -3],
        # pos=[0, 0, 0],
        # rot=[0.707, 0, 0.707, 0],
        rot=[1, 0, 0, 0],  # {action:global} = {x, z}, {y, x}, {z, y}
        dof_pos={
            "RevoluteJoint_x": 0.0,
            "RevoluteJoint_y": 0.0,
            "RevoluteJoint_z": 0.0,
            "PrismaticJoint": 0.0,
            "RevoluteJoint_right" : 0.0,
            "RevoluteJoint_left" : 0.0,
        },
        dof_vel={".*": 0.0},

    ),
    ee_info=SingleArmManipulatorCfg.EndEffectorFrameCfg(
        body_name="Xform_d", pos_offset=(0.0, 0.0, 0.0), rot_offset=(1.0, 0.0, 0.0, 0.0)  # xform / head
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
        "wrist_rev": ActuatorGroupCfg(
            dof_names=["RevoluteJoint_[x-z]"],
            model_cfg=ImplicitActuatorCfg(velocity_limit=VELOCITY_LIMIT, torque_limit=TORQUE_LIMIT),
            control_cfg=ActuatorControlCfg(
                command_types=["p_abs"],
                stiffness={".*": STIFFNESS},
                damping={".*": DAMPING},
                dof_pos_offset={
                    "RevoluteJoint_x": 0.0,
                    "RevoluteJoint_y": 0.0,
                    "RevoluteJoint_z": 0.0,
                },
            ),
        ),
        "wrist_trans": ActuatorGroupCfg(
            dof_names=["PrismaticJoint"],
            model_cfg=ImplicitActuatorCfg(velocity_limit=VELOCITY_LIMIT, torque_limit=TORQUE_LIMIT),
            control_cfg=ActuatorControlCfg(
                command_types=["p_abs"],
                stiffness={".*": STIFFNESS},
                damping={".*": DAMPING},
                dof_pos_offset={"PrismaticJoint": 0.0},
            ),
        ),
        "opener": GripperActuatorGroupCfg(
            dof_names=["RevoluteJoint_right", "RevoluteJoint_left"],
            model_cfg=ImplicitActuatorCfg(velocity_limit=VELOCITY_LIMIT, torque_limit=TORQUE_LIMIT),
            control_cfg=ActuatorControlCfg(command_types=["p_abs"], stiffness={".*": 2 * STIFFNESS}, damping={".*": 2 * DAMPING}),
            mimic_multiplier={"RevoluteJoint_right": -1, "RevoluteJoint_left": 1},
            open_dof_pos=0,
            close_dof_pos=3.141592 * (120 / 180),
        )
    },
)
