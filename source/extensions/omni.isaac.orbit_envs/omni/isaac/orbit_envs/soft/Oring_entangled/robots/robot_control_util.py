
###
#  gripper attach module for ring object RL env
# 23.05.23
###

import argparse
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

import numpy as np
import os
import random
import torch

# import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core import World

# from omni.isaac.core.simulation_context import SimulationContext
from pxr import UsdGeom, Usd, Gf, PhysicsSchemaTools, Sdf, PhysxSchema

# from omni.isaac.core.utils.stage import add_reference_to_stage, get_current_stage
from omni.isaac.core.articulations.articulation_view import ArticulationView
from omni.isaac.core.prims import GeometryPrimView
from omni.isaac.core.utils.stage import open_stage, add_reference_to_stage, get_current_stage
from omni.physx import get_physx_scene_query_interface 

from omni.isaac.core.utils.prims import create_prim, delete_prim, get_prim_at_path
from omni.isaac.core.physics_context.physics_context import PhysicsContext
import math

class RobotUtils():
    def __init__(self):
        self.hit_prim = []
    
    def set_attach(self, rigid, deform):
        """
        define deformable, rigid_randomization
        """
        rigid_prim = get_prim_at_path(rigid)
        deform_path = UsdGeom.Mesh.Get(get_current_stage(), deform)
        attachment_path = deform_path.GetPath().AppendElementString("attachment")
        attachment = PhysxSchema.PhysxPhysicsAttachment.Define(
            get_current_stage(), attachment_path
        )
        attachment.GetActor0Rel().SetTargets([deform_path.GetPath()])
        attachment.GetActor1Rel().SetTargets([rigid_prim.GetPath()])
        PhysxSchema.PhysxAutoAttachmentAPI.Apply(attachment.GetPrim())
            
    def report_hit(self, hit):
        self.hit_prim.append(hit.collision)
        return True