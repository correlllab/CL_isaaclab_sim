
import numpy as np
import json
import sys

import omni
import omni.replicator.core as rep
from pxr import Gf
from isaacsim.sensors.rtx import LidarRtx

if "/home/code/exts/isaac_exts/utils" not in sys.path:
    sys.path.append("/home/code/exts/isaac_exts/utils")

from utils import log_func, get_prim_transformations, euler_to_quat

with open("/home/code/exts/isaac_exts/cl_livox/cl_livox_python/livox_config.json") as f:
    try:
        sensor_attributes = json.load(f)
    except json.JSONDecodeError:
        pass

class LivoxLidar:
    def __init__(self, name: str, prim_base_path: str):
        self.name = name
        self.prim_base_path = prim_base_path
        self.render_product_path = None

    @log_func
    def initialize(self):
        stage = omni.usd.get_context().get_stage()
        position, orient_quat, _ = get_prim_transformations(stage.GetPrimAtPath(self.prim_base_path))
        print(f"{position=}")
        print(f"{orient_quat=}")
        orient_quat = orient_quat.tolist()
        try:
            omni_lidar_prim = rep.functional.create.omni_lidar(position, orient_quat, self.name, self.prim_base_path)
        except Exception as e:
            import traceback
            traceback.print_exc()
            print(e)
            raise RuntimeError
        sensor = LidarRtx(prim_path = self.name + self.path, translation = position, orientation = orient_euler, config_file_name = "Example_Rotary", **sensor_attributes,)
        sensor.initialize()

        rp = rep.create_render_product(sensor.prim_path, [1, 1])
        self.render_product_path = rp.path
        sensor.attach_annotator("IsaacCreateRTXLidarScanBuffer")
        return self

    @log_func
    def _publish_data(self):
        writer = rep.writers.get("RTXLidar" + "ROS2PublishPointCloud")
        writer.initialize(
            topicName="/livox/lidar",
            frameId="lidar_link",
                )
        writer.attach([self.render_product_path])
        return self


        



