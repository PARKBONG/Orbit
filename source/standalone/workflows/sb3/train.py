# Copyright (c) 2022, NVIDIA CORPORATION & AFFILIATES, ETH Zurich, and University of Toronto
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to train RL agent with Stable Baselines3."""

"""Launch Isaac Sim Simulator first."""


import argparse
import numpy as np
import os

from omni.isaac.kit import SimulationApp

from omni.isaac.orbit_envs.bong.utils import git_hash

# add argparse arguments
parser = argparse.ArgumentParser("Welcome to Orbit: Omniverse Robotics Environments!")
parser.add_argument("--headless", action="store_true", default=False, help="Force display off at all times.")
parser.add_argument("--cpu", action="store_true", default=False, help="Use CPU pipeline.")
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")
args_cli = parser.parse_args()  # Namespace(cpu=True, headless=False, num_envs=1, seed=None, task='Isaac-Humanoid-v0')

# launch the simulator
config = {"headless": args_cli.headless}
# load cheaper kit config in headless
if args_cli.headless:
    app_experience = f"{os.environ['EXP_PATH']}/omni.isaac.sim.python.gym.headless.kit"
else:
    app_experience = f"{os.environ['EXP_PATH']}/omni.isaac.sim.python.kit"
# launch the simulator
simulation_app = SimulationApp(config, experience=app_experience)

"""Rest everything follows."""


import gym
import os
from datetime import datetime

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.logger import configure
from stable_baselines3.common.vec_env import VecNormalize

from omni.isaac.orbit.utils.io import dump_pickle, dump_yaml

import omni.isaac.contrib_envs  # noqa: F401
import omni.isaac.orbit_envs  # noqa: F401
from omni.isaac.orbit_envs.utils import parse_env_cfg
from omni.isaac.orbit_envs.utils.wrappers.sb3 import Sb3VecEnvWrapper

from config import parse_sb3_cfg

# from omni.isaac.orbit_envs.bong.lift.callbacks import GripperCloseCallback


def main():
    """Train with stable-baselines agent."""
    # parse configuration
    env_cfg = parse_env_cfg(args_cli.task, use_gpu=not args_cli.cpu, num_envs=args_cli.num_envs)  # {'env': {'num_envs': 1, 'env_spacing': 5, 'episode_length': 1000, 'control_frequency_inv': 2, 'power_scale': 1.0, 'angular_velocity_scale': 0.25, 'dof_velocity_scale': 0.1, 'contact_force_scale': 0.01, 'heading_weight': 0.5, ...}, 'scene': {'humanoid': {...}}, 'sim': {'dt': 0.0083, 'substeps': 1, 'gravity': [...], 'enable_scene_query_support': False, 'use_gpu_pipeline': False, 'use_flatcache': True, 'device': 'cpu', 'physx': {...}}}
    agent_cfg = parse_sb3_cfg(args_cli.task)  # {'policy': 'MlpPolicy', 'n_timesteps': 10000000.0, 'batch_size': 256, 'n_steps': 512, 'gamma': 0.95, 'learning_rate': 3.56987e-05, 'ent_coef': 0.00238306, 'clip_range': 0.3, 'n_epochs': 5, 'gae_lambda': 0.9, 'max_grad_norm': 2, 'vf_coef': 0.431892, 'policy_kwargs': {'log_std_init': -2, 'ortho_init': False, 'activation_fn': <class 'torch.nn.modules.activation.ReLU'>, 'net_arch': [...]}}
    # override configuration with command line arguments
    if args_cli.seed is not None:
        agent_cfg["seed"] = args_cli.seed

    # directory for logging into
    log_dir = os.path.join("logs", "sb3", args_cli.task, datetime.now().strftime("%b%d_%H-%M-%S"))
    log_dir = log_dir + "_" + git_hash()
    # dump the configuration into log-directory
    dump_yaml(os.path.join(log_dir, "params", "env.yaml"), env_cfg)
    dump_yaml(os.path.join(log_dir, "params", "agent.yaml"), agent_cfg)
    dump_pickle(os.path.join(log_dir, "params", "env.pkl"), env_cfg)
    dump_pickle(os.path.join(log_dir, "params", "agent.pkl"), agent_cfg)

    # read configurations about the agent-training
    policy_arch = agent_cfg.pop("policy")  # 'MlpPolicy'
    n_timesteps = agent_cfg.pop("n_timesteps")

    # create isaac environment
    env = gym.make(args_cli.task, cfg=env_cfg, headless=args_cli.headless)  # very important
    # wrap around environment for stable baselines
    env = Sb3VecEnvWrapper(env)
    # set the seed
    env.seed(seed=agent_cfg["seed"])

    if "normalize_input" in agent_cfg:
        env = VecNormalize(
            env,
            training=True,
            norm_obs="normalize_input" in agent_cfg and agent_cfg.pop("normalize_input"),
            norm_reward="normalize_value" in agent_cfg and agent_cfg.pop("normalize_value"),
            clip_obs="clip_obs" in agent_cfg and agent_cfg.pop("clip_obs"),
            gamma=agent_cfg["gamma"],
            clip_reward=np.inf,
        )

    # create agent from stable baselines
    agent = PPO(policy_arch, env, verbose=1, **agent_cfg)  # agent includes env
    # configure the logger
    new_logger = configure(log_dir, ["stdout", "tensorboard"])
    agent.set_logger(new_logger)

    # callbacks for agent
    checkpoint_callback = CheckpointCallback(save_freq=100, save_path=log_dir, name_prefix="model", verbose=2)
    # gripperclose_callback = GripperCloseCallback(threshold=0.1)
    # train the agent
    # agent.learn(total_timesteps=n_timesteps)  # sb3.py/reset
    agent.learn(total_timesteps=n_timesteps, callback=checkpoint_callback)  # sb3.py/reset
    # agent.learn(total_timesteps=n_timesteps, callback=gripperclose_callback)  # sb3.py/reset
    # save the final model
    agent.save(os.path.join(log_dir, "model"))

    # close the simulator
    env.close()
    simulation_app.close()

    # slack_notification()


if __name__ == "__main__":
    main()
