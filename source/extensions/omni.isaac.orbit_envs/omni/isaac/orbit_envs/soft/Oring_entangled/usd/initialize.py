
###
# Initialization for ring object RL env
# 23.05.20
###

import argparse
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

import numpy as np
import os
import random
import torch

import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core import World

from omni.isaac.core.simulation_context import SimulationContext
from omni.isaac.core.utils.viewports import set_camera_view
from pxr import Gf, UsdGeom

# from orbit repo
from pxr import UsdGeom, PhysxSchema
from omni.isaac.core.utils.stage import add_reference_to_stage, get_current_stage
from omni.isaac.core.articulations.articulation_view import ArticulationView
from omni.isaac.core.utils.stage import open_stage
import math

class Test():
    def __init__(self):
        # self._device = "cuda:0"
        self._path = os.getcwd()

    def revolute_pole(self):
        # self.pole.set_joint_velocities(velocities=[4], joint_indices=[1])
        self.pole.set_joint_velocity_targets(velocities=[3], joint_indices=[1])
    
    def stop_pole(self):
        self.pole.set_joint_velocity_targets(velocities=[0], joint_indices=[1])
        self.pole.set_joint_positions(positions=[0], joint_indices=[1])
    def main(self):
        object_usd_path = "omniverse://localhost/Users/chanyoung/Asset/CoRL/test_scene_2.usd"
        open_stage(usd_path=object_usd_path)
        
        world = World(stage_units_in_meters=1)
        self.world = world

        self.pole = ArticulationView(
            prim_paths_expr="/World/pole/rigid_pole", name="pole")

        # Spawn camera
        world.reset()
        # world.play()
        self.pole.initialize()
        stop = False
        i = 0
        while simulation_app.is_running():
            if world.is_playing():
                if world.current_time_step_index == 0:
                    world.reset()
            world.step(render=True)
            i += 1
            if i > 20:
                aa = self.pole.get_joint_positions()                
                if math.degrees(aa[:, 1]) < -359:
                    stop = True
                
                if stop:
                    self.stop_pole()
                    print("stop it")
                else:
                    self.revolute_pole()
                    
                print("sibal", math.degrees(aa[:, 1]))
                
if __name__ == "__main__":
    try:
        test = Test()
        test.main()
    except Exception as e:
        import traceback
        traceback.print_exc()
    finally:
        simulation_app.close()
