from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core.prims import XFormPrimView
import omni.usd
from pxr import UsdGeom, PhysxSchema
import numpy as np
import torch

class PointCloudHandle():
    '''This is collision based pcd handle'''

    def __init__(self, deformable_path):
        self.ORIENT = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
        # self.SCALE = np.array([[0.02, 0.02, 0.02]])

        # deformable_path = "/Root/ORG60_5x/orin/orin"
        deformable_prim = get_current_stage().GetPrimAtPath(deformable_path)
        self.deformable_body = PhysxSchema.PhysxDeformableBodyAPI(deformable_prim)
        self.deformable_body_xform = XFormPrimView(prim_paths_expr=deformable_path, name="Deformable_pcd")
        # mesh_indices = deformable_body.GetCollisionIndicesAttr().Get()

    def get(self):
        local_collision_point = (np.array(self.deformable_body.GetCollisionPointsAttr().Get())) @ self.ORIENT.T
        global_collision_point = self.deformable_body_xform.get_world_poses()[0]
        self.point_cloud_position = torch.from_numpy(local_collision_point) + global_collision_point  # This is what you want!
        return self.point_cloud_position
    
    def visualizer_setup(self, points_path="/World/Points", color=(1, 0, 0), size=0.1):
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

    def partial_visualizer_setup(self, points_path="/World/Points", color=(0, 0, 1), size=0.1, sample_size=10):
        # N, _ = np.array(self.deformable_body.GetCollisionPointsAttr().Get()).shape

        self.get()
        vis_points = int(self.point_cloud_position.numpy().shape[0] / sample_size)

        # Generate indices
        self.partial_indices = np.linspace(0, self.point_cloud_position.numpy().shape[0] - 1, vis_points, dtype=int)

        partial_N = self.partial_indices.shape[0]
        point_list = np.zeros([partial_N, 3])
        sizes = size * np.ones(partial_N)
        stage = omni.usd.get_context().get_stage()
        self.points = UsdGeom.Points.Define(stage, points_path)
        self.points.CreatePointsAttr().Set(point_list)
        self.points.CreateWidthsAttr().Set(sizes)
        self.points.CreateDisplayColorPrimvar("constant").Set([color])
        # return points

    def partial_visualizer_update(self):
        self.get()
        self.points.GetPointsAttr().Set(self.point_cloud_position.numpy()[self.partial_indices])  # vis