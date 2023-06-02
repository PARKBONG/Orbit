# Copyright (c) 2022, NVIDIA CORPORATION & AFFILIATES, ETH Zurich, and University of Toronto
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Environment for lifting objects with fixed-arm robots."""

from .oring_cfg import OringEnvCfg
from .oring_env import OringEnv
# from .oring_util import PointCloudHandle

__all__ = ["OringEnv", "OringEnvCfg"]
