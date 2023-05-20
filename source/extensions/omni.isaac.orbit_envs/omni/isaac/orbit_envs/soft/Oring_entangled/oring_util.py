from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core.prims import XFormPrimView
import omni.usd
from pxr import UsdGeom, PhysxSchema
import numpy as np
import torch

class PointCloudHandle():
    '''This is collision based pcd handle'''

    def __init__(self, deformable_path):
        # self.ORIENT = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        self.SCALE = np.array([[0.02, 0.02, 0.02]])

        # deformable_path = "/Root/ORG60_5x/orin/orin"
        deformable_prim = get_current_stage().GetPrimAtPath(deformable_path)
        self.deformable_body = PhysxSchema.PhysxDeformableBodyAPI(deformable_prim)
        self.deformable_body_xform = XFormPrimView(prim_paths_expr=deformable_path, name="Deformable_pcd")
        # mesh_indices = deformable_body.GetCollisionIndicesAttr().Get()

    def get(self):
        local_collision_point = (self.SCALE * np.array(self.deformable_body.GetCollisionPointsAttr().Get()))  # @ self.ORIENT.T
        global_collision_point = self.deformable_body_xform.get_world_poses()[0]
        self.point_cloud_position = torch.from_numpy(local_collision_point) + global_collision_point  # This is what you want!
        return self.point_cloud_position
    
    def visualizer_setup(self, points_path="/World/Points", color=(1, 0, 0), size=0.2):
        N, _ = np.array(self.deformable_body.GetCollisionPointsAttr().Get()).shape

        point_list = np.zeros([N, 3])
        sizes = size * np.ones(N)
        stage = omni.usd.get_context().get_stage()
        self.points = UsdGeom.Points.Define(stage, points_path)
        self.points.CreatePointsAttr().Set(point_list)
        self.points.CreateWidthsAttr().Set(sizes)
        self.points.CreateDisplayColorPrimvar("constant").Set([color])
        # return points

    def visualizer_update(self):
        self.get()
        self.points.GetPointsAttr().Set((self.point_cloud_position.numpy()))  # vis