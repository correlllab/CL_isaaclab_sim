import gc
import omni.ext
from .realsense import Realsense, CamType

robot_base = "/World/envs/env_0/Robot/h1_2_26dof_with_inspire_rev_1_0_with_CL_realsense"
l_base = f"{robot_base}/L_hand_base_link/CL_L_realsense/rsd455/RSD455"
r_base = f"{robot_base}/R_hand_base_link/CL_R_realsense/rsd455/RSD455"

cameras = [("l_depth", f"{l_base}/Camera_Pseudo_Depth", CamType.DEPTH), ("l_rgb", f"{l_base}/Camera_OmniVision_OV9782_Color", CamType.RGB), ("r_depth", f"{r_base}/Camera_Pseudo_Depth", CamType.DEPTH), ("r_rgb", f"{r_base}/Camera_OmniVision_OV9782_Color", CamType.RGB)]

class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):

        print("/n/n/n")
        for (name, path, role) in cameras:
            cam = Realsense(name, path, role).initialize().publish()

    def on_shutdown(self):
        gc.collect()
