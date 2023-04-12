import torch

from stable_baselines3.common.callbacks import BaseCallback
# from omni.isaac.orbit_envs.bong.lift.bong_lift_env import LiftRewardManager


class GripperCloseCallback(BaseCallback):
    def __init__(self, threshold):
        super(GripperCloseCallback, self).__init__()
        self.threshold = threshold

    def _on_step(self):
        # bong = self.training_env
        print(torch.sum(torch.square(self.training_env.robot.data.ee_state_w[:, 0:3] - self.training_env.object.data.root_pos_w), dim=1))
        #close griiper
        
        # target_position = self.training_env.target_position
        # ee_position = self.training_env.ee_position
        # distance = np.linalg.norm(target_position - ee_position)

        # if distance < self.threshold:
        # self.training_env.close_gripper()
        # Optionally, you can set a flag to end the episode early
        # self.training_env.set_done(True)

        return True
