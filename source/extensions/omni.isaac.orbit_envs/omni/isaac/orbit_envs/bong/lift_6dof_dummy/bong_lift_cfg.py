# Copyright (c) 2022, NVIDIA CORPORATION & AFFILIATES, ETH Zurich, and University of Toronto
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from omni.isaac.orbit.controllers.differential_inverse_kinematics import DifferentialInverseKinematicsCfg
from omni.isaac.orbit.objects import RigidObjectCfg
# from .robots.franka import FRANKA_PANDA_ARM_WITH_PANDA_HAND_CFG
from .robots.robotiq import ROBOTIQ_WRIST_WITH_ROBOTIQ_CFG
from omni.isaac.orbit.robots.single_arm import SingleArmManipulatorCfg
from omni.isaac.orbit.utils import configclass
from omni.isaac.orbit.utils.assets import ISAAC_NUCLEUS_DIR

from omni.isaac.orbit_envs.isaac_env_cfg import EnvCfg, IsaacEnvCfg, PhysxCfg, SimCfg, ViewerCfg

##
# Scene settings
##


@configclass
class TableCfg:
    """Properties for the table."""

    # note: we use instanceable asset since it consumes less memory
    usd_path = f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd"


# @configclass
# class PointCfg:
#     """Properties for the point."""

#     # note: we use instanceable asset since it consumes less memory
#     usd_path = f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd"


@configclass
class ManipulationObjectCfg(RigidObjectCfg):
    """Properties for the object to manipulate in the scene."""
    my_dir = "/home/bong/.local/share/ov/pkg/isaac_sim-2022.2.1/Orbit/source/extensions/omni.isaac.orbit_envs/omni/isaac/orbit_envs"
    meta_info = RigidObjectCfg.MetaInfoCfg(
        usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/DexCube/dex_cube_instanceable.usd",
        # usd_path=my_dir + "/sphere_col.usd",
        scale=(1, 1, 2),
    )
    init_state = RigidObjectCfg.InitialStateCfg(
        pos=(0.4, 0.0, 0.075), rot=(1.0, 0.0, 0.0, 0.0), lin_vel=(0.0, 0.0, 0.0), ang_vel=(0.0, 0.0, 0.0)
    )
    rigid_props = RigidObjectCfg.RigidBodyPropertiesCfg(
        solver_position_iteration_count=16,
        solver_velocity_iteration_count=1,
        max_angular_velocity=1000.0,
        max_linear_velocity=1000.0,
        max_depenetration_velocity=5.0,
        disable_gravity=False,
    )
    # collision_props = RigidObjectCfg.CollisionPropertiesCfg(
    #     collision_enabled=True

    # )
    physics_material = RigidObjectCfg.PhysicsMaterialCfg(
        static_friction=0.5, dynamic_friction=0.5, restitution=0.0, prim_path="/World/Materials/cubeMaterial", density=0.001
    )


# @configclass
# class VisualObjectCfg(RigidObjectCfg):
#     """Properties for the object to manipulate in the scene."""

#     meta_info = RigidObjectCfg.MetaInfoCfg(
#         usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/DexCube/dex_cube_instanceable.usd",
#         scale=(1, 1, 1),
#     )
#     init_state = RigidObjectCfg.InitialStateCfg(
#         pos=(0.4, 0.0, 0.075), rot=(1.0, 0.0, 0.0, 0.0), lin_vel=(0.0, 0.0, 0.0), ang_vel=(0.0, 0.0, 0.0)
#     )

#     collision_props = RigidObjectCfg.CollisionPropertiesCfg(
#         collision_enabled=False
#     )
#     rigid_props = RigidObjectCfg.RigidBodyPropertiesCfg(
#         disable_gravity=True,
#     )


@configclass
class GoalMarkerCfg:
    """Properties for visualization marker."""

    # usd file to import
    usd_path = f"{ISAAC_NUCLEUS_DIR}/Props/UIElements/frame_prim.usd"
    # scale of the asset at import
    scale = [0.05, 0.05, 0.05]  # x,y,z


@configclass
class FrameMarkerCfg:
    """Properties for visualization marker."""

    # usd file to import
    usd_path = f"{ISAAC_NUCLEUS_DIR}/Props/UIElements/frame_prim.usd"
    # scale of the asset at import
    scale = [0.1, 0.1, 0.1]  # x,y,z


##
# MDP settings
##


@configclass
class RandomizationCfg:
    """Randomization of scene at reset."""

    @configclass
    class ObjectInitialPoseCfg:
        """Randomization of object initial pose."""

        # category
        position_cat: str = "uniform"  # randomize position: "default", "uniform"
        orientation_cat: str = "uniform"  # randomize position: "default", "uniform"
        # randomize position
        position_uniform_min = [0.4, -0.25, 0.030]  # position (x,y,z) z = 0.05
        position_uniform_max = [0.6, 0.25, 0.030]  # position (x,y,z)

    @configclass
    class ObjectDesiredPoseCfg:
        """Randomization of object desired pose."""

        # category
        position_cat: str = "default"  # randomize position: "default", "uniform"
        orientation_cat: str = "default"  # randomize position: "default", "uniform"
        # randomize position
        position_default = [0.4, 0, 0.45]  # position default (x,y,z)
        position_uniform_min = [0, -0.3, 0.35]  # position (x,y,z) [0, -0.25, 0.55]
        position_uniform_max = [0.8, 0.3, 0.55]  # position (x,y,z)   [0.8, 0.3, 0.55]
        # randomize orientation
        orientation_default = [1.0, 0.0, 0.0, 0.0]  # orientation default

    # initialize
    object_initial_pose: ObjectInitialPoseCfg = ObjectInitialPoseCfg()
    object_desired_pose: ObjectDesiredPoseCfg = ObjectDesiredPoseCfg()


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg:
        """Observations for policy group."""

        # global group settings
        # enable_corruption: bool = True
        enable_corruption: bool = False
        # observation terms
        # -- joint state
        arm_dof_pos = {"scale": 1.0}
        # arm_dof_pos_3D = {"scale": 1.0}
        # arm_dof_pos_scaled = {"scale": 1.0}
        # arm_dof_vel = {"scale": 0.5, "noise": {"name": "uniform", "min": -0.01, "max": 0.01}}
        arm_dof_vel = {"scale": 1.0}
        # arm_dof_vel_3D = {"scale": 1.0}
        # tool_vel = {"scale": 1.0}
        # tool_dof_pos_scaled = {"scale": 1.0}
        # -- end effector state
        # tool_positions = {"scale": 1.0}
        # tool_orientations = {"scale": 1.0}
        # -- object state
        object_positions = {"scale": 1.0}
        object_orientations = {"scale": 1.0}
        object_relative_tool_positions = {"scale": 1.0}
        object_relative_tool_orientations = {"scale": 1.0}
        # -- object desired state
        # object_desired_positions = {"scale": 1.0}
        # -- previous action
        # arm_actions = {"scale": 1.0}
        # tool_actions = {"scale": 1.0}
        # bong_is_catch = {"scale": 10}
        # bong_obj_to_desire = {"scale": 1.0}
        # bong_obj_height = {"scale": 10}
        # bong_object_ang_vel = {"scale": 1.0}
        # bong_object_lin_vel = {"scale": 1.0}
        bong_cube_pcd = {"scale": 1.0}

    # global observation settings
    return_dict_obs_in_group = False
    """Whether to return observations as dictionary or flattened vector within groups."""
    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # -- robot-centric
    reaching_object_position_l2 = {"weight": 20}
    # reaching_object_height = {"weight": 50}
    # reaching_object_position_exp = {"weight": 2.5, "sigma": 0.25}
    # reaching_object_position_tanh = {"weight": 2.5, "sigma": 0.1}
    penalizing_arm_dof_velocity_l2 = {"weight": 0.3}
    # penalizing_tool_dof_velocity_l2 = {"weight": 1}
    # penalizing_robot_dof_acceleration_l2 = {"weight": 1e-7}
    # -- action-centric
    # penalizing_arm_action_rate_l2 = {"weight": 0.5}
    # penalizing_tool_action_l2 = {"weight": 1e-2}
    # -- object-centric
    # tracking_object_position_l2 = {"weight": 1}
    # tracking_object_position_exp = {"weight": 5.0, "sigma": 0.25, "threshold": 0.08}
    # tracking_object_position_tanh = {"weight": 5.0, "sigma": 0.2, "threshold": 0.08}
    # lifting_object_success = {"weight": 3.5, "threshold": 0.08}
    # lifting_object_desired_success = {"weight" : 2}
    bong_catch_object = {"weight": 200}
    bong_object_falling = {"weight": 10}
    # bong_catch_object = {"weight": 300}
    # bong_catch_failure = {"weight": 50}
    # bong_is_success = {"weight": 1000}  #this
    # bong_robot_out_of_box = {"weight": 10}
    # bong_object_height = {"weight": 1000}  #this
    # bong_is_cheating = {"weight": 100}
    # bong_ee_to_obj_scalar = {"weight": 10}
    
@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    episode_timeout = True  # reset when episode length ended
    object_falling = True  # reset when object falls off the table
    is_success = False  # reset when object is lifted
    is_catch = True  # reset when object is lifted
    fail_to_catch = False  # reset when object is lifted
    is_obj_desired = False


@configclass
class ControlCfg:
    """Processing of MDP actions."""

    # action space
    control_type = "default"  # "default", "inverse_kinematics"
    # decimation: Number of control action updates @ sim dt per policy dt
    decimation = 1

    # configuration loaded when control_type == "inverse_kinematics"
    inverse_kinematics: DifferentialInverseKinematicsCfg = DifferentialInverseKinematicsCfg(
        command_type="pose_rel",
        ik_method="dls",
        position_command_scale=(0.1, 0.1, 0.1),
        rotation_command_scale=(0.1, 0.1, 0.1),
    )


##
# Environment configuration
##

@configclass
class LiftEnvCfg(IsaacEnvCfg):
    """Configuration for the Lift environment."""

    # General Settings
    env: EnvCfg = EnvCfg(num_envs=4096, env_spacing=2.5, episode_length_s=5.0)
    viewer: ViewerCfg = ViewerCfg(debug_vis=True, eye=(7.5, 7.5, 7.5), lookat=(0.0, 0.0, 0.0))
    # Physics settings
    sim: SimCfg = SimCfg(
        dt=0.01,
        substeps=1,
        physx=PhysxCfg(
            gpu_found_lost_aggregate_pairs_capacity=1024 * 1024 * 4,
            gpu_total_aggregate_pairs_capacity=16 * 1024,
            friction_correlation_distance=0.00625,
            friction_offset_threshold=0.01,
            bounce_threshold_velocity=0.2,
        ),
    )

    # Scene Settings
    # -- robot
    # robot: SingleArmManipulatorCfg = FRANKA_PANDA_ARM_WITH_PANDA_HAND_CFG
    robot: SingleArmManipulatorCfg = ROBOTIQ_WRIST_WITH_ROBOTIQ_CFG
    # -- object
    object: ManipulationObjectCfg = ManipulationObjectCfg()
    # -- bong
    # visual_object: VisualObjectCfg = VisualObjectCfg()
    # -- table
    table: TableCfg = TableCfg()
    # -- visualization marker
    goal_marker: GoalMarkerCfg = GoalMarkerCfg()
    frame_marker: FrameMarkerCfg = FrameMarkerCfg()

    # MDP settings
    randomization: RandomizationCfg = RandomizationCfg()
    observations: ObservationsCfg = ObservationsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    # Controller settings
    control: ControlCfg = ControlCfg()
