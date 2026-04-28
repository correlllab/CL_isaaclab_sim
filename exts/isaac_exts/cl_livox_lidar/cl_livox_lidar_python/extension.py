import gc
import omni.ext
from .livox_lidar import LivoxLidar

lidar_link_path = "/World/envs/env_0/Robot/h1_2_26dof_with_inspire_rev_1_0_with_CL_realsense/lidar_link"

class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        self.livox_lidar = LivoxLidar("lidar", lidar_link_path).initialize()._publish_data()

    def on_shutdown(self):
        gc.collect()

