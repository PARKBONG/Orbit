# Copyright (c) 2022, NVIDIA CORPORATION & AFFILIATES, ETH Zurich, and University of Toronto
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from omni.isaac.orbit.robots.single_arm import SingleArmManipulator
import torch


class Bong_SingleArmManipulator(SingleArmManipulator):

    def __init__(self, cfg):
        super().__init__(cfg)

    def apply_action_bong(self, actions: torch.Tensor, index):
            """Apply the input action for the robot into the simulator.

            The actions are first processed using actuator groups. Depending on the robot configuration,
            the groups compute the joint level simulation commands and sets them into the PhysX buffers.

            Args:
                actions (torch.Tensor): The input actions to apply.
            """
            row, col = self._data.dof_vel_targets.shape
            dof_pos_targets = torch.zeros([row, col])
            # slice actions per actuator group
            group_actions_dims = [group.control_dim for group in self.actuator_groups.values()]
            all_group_actions = torch.split(actions, group_actions_dims, dim=-1)
            # note: we use internal buffers to deal with the resets() as the buffers aren't forwarded
            #   unit the next simulation step.
            dof_pos = self._data.dof_pos
            dof_vel = self._data.dof_vel
            # process actions per group
            for group, group_actions in zip(self.actuator_groups.values(), all_group_actions):
                # compute group dof command
                control_action = group.compute(
                    group_actions, dof_pos=dof_pos[:, group.dof_indices], dof_vel=dof_vel[:, group.dof_indices]
                )
                # update targets
                if control_action.joint_positions is not None:
                    dof_pos_targets[:, group.dof_indices] = control_action.joint_positions
                # if control_action.joint_velocities is not None:
                #     dof_vel_targets[:, group.dof_indices] = control_action.joint_velocities
                # if control_action.joint_efforts is not None:
                #     dof_effort_targets[:, group.dof_indices] = control_action.joint_efforts
                # update state
                # -- torques
                # self._data.computed_torques[:, group.dof_indices] = group.computed_torques
                # self._data.applied_torques[:, group.dof_indices] = group.applied_torques
                # -- actuator data
                # self._data.gear_ratio[:, group.dof_indices] = group.gear_ratio
                if group.velocity_limit is not None:
                    self._data.soft_dof_vel_limits[:, group.dof_indices] = group.velocity_limit
            # silence the physics sim for warnings that make no sense :)
            # note (18.08.2022): Saw a difference of up to 5 ms per step when using Isaac Sim
            #   ArticulationView.apply_action() method compared to direct PhysX calls. Thus,
            #   this function is optimized to apply actions for the whole robot.

            # idx_bool_tensor

            self.articulations._physics_sim_view.enable_warnings(False)
            # apply actions into sim
            if self.sim_dof_control_modes["position"]:
                self.articulations._physics_view.set_dof_position_targets(dof_pos_targets, index)
            # if self.sim_dof_control_modes["velocity"]:
            #     self.articulations._physics_view.set_dof_velocity_targets(self._data.dof_vel_targets, self._ALL_INDICES)
            # if self.sim_dof_control_modes["effort"]:
            #     self.articulations._physics_view.set_dof_actuation_forces(self._data.dof_effort_targets, self._ALL_INDICES)
            # enable warnings for other unsafe operations ;)
            self.articulations._physics_sim_view.enable_warnings(True)