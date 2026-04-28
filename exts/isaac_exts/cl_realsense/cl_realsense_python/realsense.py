"""
hi, this are some of the classes for the realsense simulation
"""
import collections
from dataclasses import dataclass
from enum import Enum, auto
import sys

import numpy as np
import omni.usd
import omni.graph.core as og
import omni.replicator.core as rep
from isaacsim.sensors.camera import Camera
import omni.syntheticdata._syntheticdata as sd
from isaacsim.ros2.bridge import read_camera_info

if "/home/code/exts/isaac_exts/utils" not in sys.path:
    sys.path.append("/home/code/exts/isaac_exts/utils")

from utils import log_func, get_prim_transformations 

class CamType(Enum):
    DEPTH = auto()
    RGB = auto()

class Realsense:
    @log_func
    def __init__(self, name: str, path: str, role: CamType, frequency: int = 20, res_width: int = 1280, res_height: int = 720):
        self.name = name
        self.path = path
        self.role = role
        self.frequency = frequency
        self.res_width = res_width
        self.res_height = res_height
        self.step_size = int(60 / self.frequency)
        self.queue_size = 10

        self.camera = None
        self.rp_path = None

    @log_func
    def initialize(self):
        stage = omni.usd.get_context().get_stage()
        position, orientation, _ = get_prim_transformations(stage.GetPrimAtPath(self.path))

        rp = rep.create.render_product(self.path, (self.res_width, self.res_height))
        self.rp_path = rp.path

        self.camera = Camera(
            self.path,
            name=self.name,
            frequency=self.frequency,
            resolution=(self.res_width, self.res_height),
            render_product_path=self.rp_path,
            position=position,
            orientation=orientation,
        )
        return self

    @log_func
    def publish(self):
        self._publish_data()
        self._publish_camera_info()
        return self

    @log_func
    def _publish_data(self):
        rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
            sd.SensorType.DistanceToImagePlane.name
        )

        if self.role == CamType.DEPTH:
            fetch = "ROS2PublishPointCloud"
            topic = "/aligned_depth_to_color/image_raw/test_pc"
            #topic = "/aligned_depth_to_color/image_raw/compressedDepth"
        elif self.role == CamType.RGB:
            fetch = "ROS2PublishImage"
            topic = "color/image_raw/test_img"
            #topic = "/color/image_raw/compressed"

        writer = rep.writers.get(rv + fetch)
        writer.initialize(
            frameId=self.name,
            nodeNamespace=self.name,
            queueSize=self.queue_size,
            topicName=topic,
        )
        writer.attach([self.rp_path])
        gate_path = omni.syntheticdata.SyntheticData._get_node_path(
            rv + "IsaacSimulationGate", self.rp_path
        )
        og.Controller.attribute(gate_path + ".inputs:step").set(self.step_size)
        return self

    @log_func
    def _publish_camera_info(self):
        if self.role == CamType.DEPTH:
            topic = "/aligned_depth_to_color/camera_info"
        elif self.role == CamType.RGB:
            topic = "/color/camera_info"

        camera_info, _ = read_camera_info(self.rp_path)
        writer = rep.writers.get("ROS2PublishCameraInfo")
        writer.initialize(
            frameId=self.name,
            nodeNamespace=self.name,
            queueSize=self.queue_size,
            topicName=topic,
            width=camera_info.width,
            height=camera_info.height,
            projectionType=camera_info.distortion_model,
            k=camera_info.k.reshape([1, 9]),
            r=camera_info.r.reshape([1, 9]),
            p=camera_info.p.reshape([1, 12]),
            physicalDistortionModel=camera_info.distortion_model,
            physicalDistortionCoefficients=camera_info.d,
        )
        writer.attach([self.rp_path])
        gate_path = omni.syntheticdata.SyntheticData._get_node_path(
            "PostProcessDispatchIsaacSimulationGate", self.rp_path
        )
        og.Controller.attribute(gate_path + ".inputs:step").set(self.step_size)
        return self

