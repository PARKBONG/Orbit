# Copyright (c) 2022, NVIDIA CORPORATION & AFFILIATES, ETH Zurich, and University of Toronto
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

####################
#  2023.05.31     #
#  Oring Twist env #
####################
# only one environment version

import gym.spaces
import math
import numpy as np
import torch
from typing import List

import random
import os

import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core.articulations.articulation_view import ArticulationView
from omni.isaac.core.prims import GeometryPrimView
from omni.isaac.core.utils.stage import add_reference_to_stage, get_current_stage
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.physx import get_physx_scene_query_interface 

import omni.isaac.orbit.utils.kit as kit_utils
from omni.isaac.orbit.markers import StaticMarker
from omni.isaac.orbit.utils.dict import class_to_dict
# from omni.isaac.orbit.utils.math import quat_inv, quat_mul, random_orientation, sample_uniform, scale_transform
from omni.isaac.orbit.utils.mdp import ObservationManager, RewardManager
from omni.isaac.orbit.robots.single_arm import SingleArmManipulator
from omni.isaac.orbit_envs.isaac_env import IsaacEnv, VecEnvIndices, VecEnvObs

from .oring_cfg import OringEnvCfg, RandomizationCfg
from .utils.prim_utils import delete_prim
from .utils.pcd_utils import PointCloudHandle
# from .oring_util import PointCloudHandle
from .models import PCN  # PCN

from pxr import UsdGeom, Usd, Gf, PhysicsSchemaTools, Sdf, PhysxSchema
# from omni.isaac.cloner import GridCloner

class OringEnv(IsaacEnv):
    """Environment for lifting an object off a table with a single-arm manipulator."""

    def __init__(self, cfg: OringEnvCfg = None, headless: bool = False):
        self.cfg = cfg
        self._path = os.getcwd()
        self._usd_path = self._path + "/source/extensions/omni.isaac.orbit_envs/omni/isaac/orbit_envs/soft/Oring_entangled/usd"
        
        # self._pre_process_cfg()
        
        self.robot = SingleArmManipulator(cfg=self.cfg.robot)
        # PCN
        self.model = PCN(16384, 256, 4).to("cuda:0")
        self.model.load_state_dict(torch.load("/home/bong/.local/share/ov/pkg/isaac_sim-2022.2.1/Orbit/source/extensions/omni.isaac.orbit_envs/omni/isaac/orbit_envs/soft/Oring_entangled/models/checkpoint/best_l1_cd_397.pth"))  # with best cd loss
        # self.model.load_state_dict(torch.load("/home/bong/.local/share/ov/pkg/isaac_sim-2022.2.1/Orbit/source/extensions/omni.isaac.orbit_envs/omni/isaac/orbit_envs/soft/Oring_entangled/models/checkpoint/checkpoint_400.pth"))  # with max epoch
        super().__init__(self.cfg, headless=headless)

        # parse the configuration for information
        self._process_cfg()
        self._initialize_views()

        # prepare the observation manager
        self._observation_manager = ObservationManager(class_to_dict(self.cfg.observations), self, self.device)
        self._reward_manager = RewardManager(
            class_to_dict(self.cfg.rewards), self, self.num_envs, self.dt, self.device
        )
        # print information about MDP
        print("[INFO] Observation Manager:", self._observation_manager)
        print("[INFO] Reward Manager: ", self._reward_manager)

        num_obs = self._observation_manager.group_obs_dim["policy"][0]
        self.observation_space = gym.spaces.Box(low=-math.inf, high=math.inf, shape=(num_obs,))

        # compute the action space
        self.action_space = gym.spaces.Box(low=-np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1]),  # r, p, y, d
                                           high=np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1]),
                                           shape=(self.num_actions,))
        print("[INFO]: Completed setting up the environment...")

        # Take an initial step to initialize the scene.
        self.sim.step()

        self.action_clip = torch.tensor([[1, 1, 1, -torch.pi, -torch.pi, -torch.pi],  # r, p, y, d
                                         [1, 1, 1, torch.pi, torch.pi, torch.pi]])

        self.deform_path = self.template_env_ns + '/oring_env/oring_01_05/oring'

    """
    Implementation specifics.
    """

    def _design_scene(self) -> List[str]:
        # ground plane
        self.ground_path = self._usd_path + "/default_ground.usd"
        kit_utils.create_ground_plane("/World/defaultGroundPlane",
                                      z_position=-2.,
                                      usd_path=self.ground_path)
        
        self.scene_path = self._usd_path + "/oring_task_env_01_05.usd"  # large
        for i in range(self.num_envs):

            prim_utils.create_prim(prim_path=f"/World/envs/env_{i}/oring_env",
                                usd_path=self.scene_path,
                                translation=[0., 0., 0.],
                                orientation=[1., 0., 0., 0.],
                                scale=[1., 1., 1.])

            self.robot.spawn(f"/World/envs/env_{i}/Robot")
        
        self.pcd = PointCloudHandle(deform_path="/World/envs/.*/oring_env/oring_01_05/oring")
        self.pcd.visualizer_setup()
        if self.cfg.viewer.debug_vis and self.enable_render:
            # create point instancer to visualize the goal points
            self._goal_markers = StaticMarker(
                "/Visuals/object_goal",
                self.num_envs,
                usd_path=self.cfg.goal_marker.usd_path,
                scale=self.cfg.goal_marker.scale,
            )
            # create marker for viewing end-effector pose
            self._ee_markers = StaticMarker(
                "/Visuals/ee_current",
                self.num_envs,
                usd_path=self.cfg.frame_marker.usd_path,
                scale=self.cfg.frame_marker.scale,
            )
            # create marker for viewing command (if task-space controller is used)
            if self.cfg.control.control_type == "inverse_kinematics":
                self._cmd_markers = StaticMarker(
                    "/Visuals/ik_command",
                    self.num_envs,
                    usd_path=self.cfg.frame_marker.usd_path,
                    scale=self.cfg.frame_marker.scale,
            )
        return ["/World/defaultGroundPlane"]

    def _reset_idx(self, env_ids: VecEnvIndices):
        # randomize the MDP
        
        # fill extras with episode information
        self.extras["episode"] = dict()
        self._reward_manager.reset_idx(env_ids, self.extras["episode"])
        self._observation_manager.reset_idx(env_ids)
        self.previous_actions[env_ids] = 0
        self.reset_buf[env_ids] = 0
        self.episode_length_buf[env_ids] = 0

        # Initialize
        for i in range(self.num_envs):
            delete_prim(f"/World/envs/env_{i}/oring_env")  # https://forums.developer.nvidia.com/t/delete-prim-in-isaac-sim/246025
            delete_prim(f"/World/envs/env_{i}/rigid_pole")

            prim_utils.create_prim(prim_path=f"/World/envs/env_{i}/oring_env",
                                usd_path=self.scene_path,
                                translation=[0., 0., 0.],
                                orientation=[1., 0., 0., 0.],
                                scale=[1., 1., 1.])

        # Deformable mover
        self.hook = ArticulationView(
            prim_paths_expr="/World/envs/.*/oring_env/pole_env/init_hook",
            name="hook")
        
        self._initialize_views()
        self.hook.initialize()
        self.joint_x = self.hook.get_dof_index("pris_x")
        self.joint_y = self.hook.get_dof_index("pris_y")
        self.joint_z = self.hook.get_dof_index("pris_z")
        self.rev_x = self.hook.get_dof_index("rev_x")



        # Randomize oring state
        # get rigid_pole
        self.get_rigid_pole()
        self.get_reset()

        # Initialize pointcloud extractor
        # self.pcd.set_off_render()
        self.pcd.initialize()

    def _step_impl(self, actions: torch.Tensor):
        self.hit_prim = []
        
        self.actions = actions.clone().to(device=self.device)

        self.robot_actions += self.actions

        for _ in range(self.cfg.control.decimation):

            self.robot.apply_action(self.robot_actions)  # robot_action

            # check touch object
            self.sim.step(render=self.enable_render)
            # check that simulation is playing
            # self.pcd.get_deform_point(render=True)
            norm_pcds, denorm_factors = self.pcd.get_deform_point()
            self.pcd.get_latent_vector(norm_pcds=norm_pcds,
                                       denorm_factors=denorm_factors,
                                       decoder=True)
            # PCN model 
            # latent vector / pointcloud
            # self.model.forward(xyz)


            if self.sim.is_stopped():
                return

        self.extras["time_outs"] = self.episode_length_buf >= self.max_episode_length
        self.robot.update_buffers(self.dt)

        self.reward_buf = self._reward_manager.compute()
        
        self._check_termination()
        
        self.previous_actions = self.actions.clone()
        
        if self.cfg.viewer.debug_vis and self.enable_render:
            self._debug_vis()

    def _get_observations(self) -> VecEnvObs:
        # compute observations
        return self._observation_manager.compute()

    """
    Helper functions - Scene handling.
    """

    def _process_cfg(self) -> None:
        """Post processing of configuration parameters."""
        # compute constants for environment
        self.dt = self.cfg.control.decimation * self.physics_dt  # control-dt
        self.max_episode_length = math.ceil(self.cfg.env.episode_length_s / self.dt) # chanyoung
        # self.max_episode_length = 10000

    def _initialize_views(self) -> None:
        """Creates views and extract useful quantities from them."""
        self.sim.reset()
        # self.num_actions = 6
        self.robot.initialize(self.env_ns + "/.*/Robot")
        self.num_actions = self.robot.num_actions

        # Articulation initialize
        # history
        self.actions = torch.zeros((self.num_envs, self.num_actions), device=self.device)
        self.previous_actions = torch.zeros((self.num_envs, self.num_actions), device=self.device)
        # robot joint actions
        self.robot_actions = torch.zeros((self.num_envs, self.robot.num_actions), device=self.device)
        # commands


    def _debug_vis(self):
        """Visualize the environment in debug mode."""
        self._ee_markers.set_world_poses(self.robot.data.ee_state_w[:, 0:3], self.robot.data.ee_state_w[:, 3:7])

    """
    Helper functions - MDP.
    """
    def _check_termination(self) -> None:
        self.reset_buf[:] = 0
        # -- episode length
        if self.cfg.terminations.episode_timeout:
            self.reset_buf = torch.where(self.episode_length_buf >= self.max_episode_length, 1, self.reset_buf)



    """
    Asset fuctions
    """
    def get_rigid_pole(self):
        rigid_path = self._usd_path + "/rigid_pole.usd"

        for i in range(self.num_envs):
            add_reference_to_stage(rigid_path, f"/World/envs/env_{i}/rigid_pole")
        # add_reference_to_stage(rigid_path, "/World/envs/env_1/rigid_pole")

        # init_positions = torch.zeros((self.num_envs, 3))
        init_positions = torch.zeros((self.num_envs, 3))
        for i in range(self.num_envs):
            init_positions[i, 1] = random.uniform(1.2, 1.8)
        self.init_rigid_pos = init_positions
        # "/World/envs/.*/Ant/torso
        GeometryPrimView(
            prim_paths_expr="/World/envs/.*/rigid_pole",
            translations=init_positions,
            
        )
        visible = torch.ones((self.num_envs, 1), dtype=torch.bool)
        self.rigid_pole = GeometryPrimView(
            prim_paths_expr="/World/envs/.*/rigid_pole/pole",
            visibilities=visible,
        )

    def get_reset(self):
        # self.rigid_pole.enable_collision()
        random_angle_list = [-6.0, -3.14, 3.14, 6.0]
        INIT_ANG_LIST = torch.as_tensor([random_angle_list[random.randint(0, 3)] for _ in range(self.num_envs)])
        _state = torch.zeros((self.num_envs))
        _state_2 = torch.where(INIT_ANG_LIST < 0, False, True)
        _break = torch.zeros((self.num_envs))

        while True:        
            self.sim.step()
            hook_joint_pos = self.hook.get_joint_positions()
            
            if all(_break):
                break

            if not all(_state):
                for i in range(self.num_envs):
                    if hook_joint_pos[i, self.joint_x] < float(self.init_rigid_pos[i, 1]) - 0.2:
                        hook_joint_pos[i, self.joint_x] += 0.01
                    else:
                        _state[i] = True
                self.hook.set_joint_positions(positions=hook_joint_pos)

            else:
                for i in range(self.num_envs):
                    if _state_2[i] == False:
                        hook_joint_pos[i, self.joint_y] = 0
                        hook_joint_pos[i, self.joint_z] = 0
                        
                        if hook_joint_pos[i, self.rev_x] > INIT_ANG_LIST[i]:
                            hook_joint_pos[i, self.rev_x] -= 0.5
                            if hook_joint_pos[i, self.rev_x] < -6.28: # -2pi
                                hook_joint_pos[i, self.rev_x] = INIT_ANG_LIST[i] - 0.1
                        else:
                            self.rigid_pole.enable_collision(indices=[i])
                            _break[i] = True
                    elif _state_2[i] == True: # 3.14, 6.28
                        hook_joint_pos[i, self.joint_y] = 0
                        hook_joint_pos[i, self.joint_z] = 0
                        
                        if hook_joint_pos[i, self.rev_x] < INIT_ANG_LIST[i]:
                            hook_joint_pos[i, self.rev_x] += 0.5
                            if hook_joint_pos[i, self.rev_x] > 6.28: # 2pi
                                hook_joint_pos[i, self.rev_x] = INIT_ANG_LIST[i]+ 0.1
                        else:
                            self.rigid_pole.enable_collision(indices=[i])
                            _break[i] = True
                    self.hook.set_joint_position_targets(positions=hook_joint_pos)

        for i in range(self.num_envs):
            delete_prim(f"/World/envs/env_{i}/oring_env/pole_env/init_hook")

        print("delete Init_hook!")


class ObservationManager(ObservationManager):
    """Reward manager for single-arm reaching environment."""

    def arm_dof_pos(self, env: OringEnv):
        """DOF positions for the arm."""
        return env.robot.data.arm_dof_pos

    def arm_dof_vel(self, env: OringEnv):
        """DOF velocity of the arm."""
        return env.robot.data.arm_dof_vel

    def arm_actions(self, env: OringEnv):
        """Last arm actions provided to env."""
        return env.actions
    
    def tool_positions(self, env: OringEnv):
        """Current end-effector position of the arm."""
        return env.robot.data.ee_state_w[:, :3] - env.envs_positions

    def tool_orientations(self, env: OringEnv):
        """Current end-effector orientation of the arm."""
        # make the first element positive
        quat_w = env.robot.data.ee_state_w[:, 3:7]
        quat_w[quat_w[:, 0] < 0] *= -1
        return quat_w

    def tool_actions(self, env: OringEnv):
        """Check grasp oring """
        return 
    
    # def pcn_latent(self, env: OringEnv):
    #     # print(env.model.get_latent(xyz=env.pcd.position).to("cpu"))
    #     return env.model.get_latent(xyz=env.pcd.position).to("cpu")  # N X 3, array
    
class RewardManager(RewardManager):
    """Reward manager for single-arm object lifting environment."""

    def penalizing_arm_dof_velocity_l2(self, env: OringEnv):
        """Penalize large movements of the robot arm."""
        return -torch.sum(torch.square(env.robot.data.arm_dof_vel), dim=1)

    def penalizing_tool_dof_velocity_l2(self, env: OringEnv):
        """Penalize large movements of the robot tool."""
        return -torch.sum(torch.square(env.robot.data.tool_dof_vel), dim=1)

    def penalizing_arm_action_rate_l2(self, env: OringEnv):
        """Penalize large variations in action commands besides tool."""
        return -torch.sum(torch.square(env.actions[:, :-1] - env.previous_actions[:, :-1]), dim=1)

    def penalizing_tool_action_l2(self, env: OringEnv):
        """Penalize large values in action commands for the tool."""
        return -torch.square(env.actions[:, -1])
