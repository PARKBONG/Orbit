# Copyright (c) 2022, NVIDIA CORPORATION & AFFILIATES, ETH Zurich, and University of Toronto
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Utilities for parsing and loading environments configurations."""


import gym
import importlib
import inspect
import os
import yaml
from typing import Any, Union

from omni.isaac.orbit.utils import update_class_from_dict, update_dict


def load_default_env_cfg(task_name: str) -> Union[dict, Any]:
    """Load default configuration file for an environment from Gym registry.

    This function resolves the configuration file for environment based on the file type.
    It supports both YAML and Python configuration files.

    Args:
        task_name (str): The name of the environment.

    Returns:
        Union[dict, Any]: The parsed configuration object.
    """
    # retrieve the configuration file to load
    cfg_entry_point: str = gym.spec(task_name)._kwargs.pop("cfg_entry_point") # 'omni.isaac.orbit_envs.classic.humanoid:humanoid_cfg.yaml'

    # parse the default config file
    if cfg_entry_point.endswith(".yaml"):
        if os.path.exists(cfg_entry_point):
            # absolute path for the config file
            config_file = cfg_entry_point
        else:
            # resolve path to the module location
            mod_name, file_name = cfg_entry_point.split(":") # 'omni.isaac.orbit_envs.classic.humanoid, humanoid_cfg.yaml
            mod_path = os.path.dirname(importlib.import_module(mod_name).__file__)
            # obtain the configuration file path
            config_file = os.path.join(mod_path, file_name)
        # load the configuration
        print(f"[INFO]: Parsing default environment configuration from: {config_file}")
        with open(config_file) as f: # config_file <- '/home/bong/.local/share/ov/pkg/isaac_sim-2022.2.1/Orbit/source/extensions/omni.isaac.orbit_envs/omni/isaac/orbit_envs/classic/humanoid/humanoid_cfg.yaml'
            cfg = yaml.full_load(f)
    else:
        if callable(cfg_entry_point):
            # resolve path to the module location
            mod_path = inspect.getfile(cfg_entry_point)
            # load the configuration
            cfg_cls = cfg_entry_point()
        else:
            # resolve path to the module location
            mod_name, attr_name = cfg_entry_point.split(":")
            mod = importlib.import_module(mod_name)
            cfg_cls = getattr(mod, attr_name)
        # load the configuration
        print(f"[INFO]: Parsing default environment configuration from: {inspect.getfile(cfg_cls)}")
        cfg = cfg_cls()

    return cfg 
#  cfg <- {'env': {'num_envs': 1024, 'env_spacing': 5, 'episode_length': 1000, 'control_frequency_inv': 2, 'power_scale': 1.0, 'angular_velocity_scale': 0.25, 'dof_velocity_scale': 0.1, 'contact_force_scale': 0.01, 'heading_weight': 0.5, ...}, 'scene': {'humanoid': {...}}, 'sim': {'dt': 0.0083, 'substeps': 1, 'gravity': [...], 'enable_scene_query_support': False, 'use_gpu_pipeline': True, 'use_flatcache': True, 'device': 'cuda:0', 'physx': {...}}}


def parse_env_cfg(task_name: str, use_gpu: bool = True, num_envs: int = None, **kwargs) -> Union[dict, Any]:
    """Parse configuration file for an environment and override based on inputs.

    Args:
        task_name (str): The name of the environment.
        use_gpu (bool, optional): Whether to use GPU/CPU pipeline. Defaults to True.
        num_envs (int, optional): Number of environments to create. Defaults to True.

    Returns:
        Union[dict, Any]: The parsed configuration object.
    """
    # create a dictionary to update from
    args_cfg = {"sim": {"physx": dict()}, "env": dict()} # {'sim': {'physx': {}}, 'env': {}}
    # resolve pipeline to use (based on input)
    if not use_gpu:
        args_cfg["sim"]["use_gpu_pipeline"] = False
        args_cfg["sim"]["physx"]["use_gpu"] = False
        args_cfg["sim"]["device"] = "cpu" # {'sim': {'physx': {...}, 'use_gpu_pipeline': False, 'device': 'cpu'}, 'env': {}}
    else:
        args_cfg["sim"]["use_gpu_pipeline"] = True
        args_cfg["sim"]["physx"]["use_gpu"] = True
        args_cfg["sim"]["device"] = "cuda:0"
    # FIXME (15.05.2022): if environment name has soft, then GPU pipeline is not supported yet
    if "soft" in task_name.lower():
        args_cfg["sim"]["use_gpu_pipeline"] = False
        args_cfg["sim"]["physx"]["use_gpu"] = True
        args_cfg["sim"]["device"] = "cpu"
        args_cfg["sim"]["use_flatcache"] = False

    # number of environments
    if num_envs is not None:
        args_cfg["env"]["num_envs"] = num_envs
        print(f"[Config]: Overriding number of environments to: {num_envs}") # {'sim': {'physx': {...}, 'use_gpu_pipeline': False, 'device': 'cpu'}, 'env': {'num_envs': 1}}

    # load the configuration
    cfg = load_default_env_cfg(task_name)
    # update the main configuration
    if isinstance(cfg, dict):
        cfg = update_dict(cfg, args_cfg)
    else:
        update_class_from_dict(cfg, args_cfg)

    # print information about pipeline
    print("[INFO]: Simulation pipeline: ", "GPU" if args_cfg["sim"]["use_gpu_pipeline"] else "CPU")
    print("[INFO]: Simulation device  : ", "GPU" if args_cfg["sim"]["physx"]["use_gpu"] else "CPU")

    return cfg
