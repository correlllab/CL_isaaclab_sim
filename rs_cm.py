#realsense camera manager class
#Written by Mateo Feit, Jan 29 2026
import omni.graph.core as og
import omni.replicator.core as rep
from isaacsim.sensors.camera import Camera
import omni.syntheticdata._syntheticdata as sd
import logging
from dataclasses import dataclass
from typing import Tuple 

logger = logging.getLogger(__name__)

@dataclass
class CameraSpecs(frozen = True):
    name: str = "Camera"
    cam_path: str = None
    #depth_path: str = base_path + "rsd455/RSD455/Camera_Pseudo_Depth"
    #rgb_path: str = base_path + "rsd455/RSD455/Camera_OmniVision_*_Color"
    frequency: int = 30
    dt: float = 1.0 / frequency
    res_width: int = 1280
    res_height: int = 720
    _stage = omni.usd.get_context().get_stage()

    def __post_init__(self):
        if self.frequency <= 0:
            raise ValueError(f"frequency: {self.frequency}; must be positive val")
        if self.res_width <= 0 or res_width >= 1920:
            raise ValueError(f"res_width: {self.res_width}; res_width must be a postive integer less than or equal to 1920")
        if self.res_height <= 0 or res_height >= 1080:
            raise ValueError(f"res_height: {self.res_height}; res_height must be a positive integer less than or equal to 1080")
        if self.dt != (1.0 / self.frequency):
            logger.warning(f"dt: {self.dt}; dt is not 1.0 / {self.frequency=}")
        if not self._stage.GetPrimAtPath(self.cam_path):
            raise ValueError(f"prim: {self.path} doenst exist on stage")
    
    def __repr__(self):
        attrs = ', '.join(f"{k}={v!r}" for k, v in vars(self).items())
        return f"{self.__class__.__name__}({attrs})"

class RealsenseCM:
    def __init__(self, specs: Tuple[CameraSpecs, CameraSpecs, CameraSpecs, CameraSpecs]):
        for spec in specs:
            init_camera(spec)

    def __repr__(self):
        return f"{specs}"

    def init_camera(self, specs):
        camera = Camera(specs.cam_path)
        camera.name = specs.name
        camera.frequency = specs.frequency
        camera.dt = specs.dt
        camera.resolution = (specs.res_width, specs.res_height)
        cam_render_product = rep.create.render_product(specs.cam_path, (specs.res_width, specs.res_height))
        camera.render_product_path = cam_render_product.path
        prim = stage.GetPrimAtPath(specs.cam_path)
        self.camera.position, self.camera.orientation, self.camera.translation = get_pos_orient(prim)
        if "Depth" in specs.cam_path.split("/")[-1]:
            publish_pointcloud_from_depth(camera)
        elif "Color" in specs.cam_path.split("/")[-1]:
            publish_rgb_stream(camera)

    @staticmethod
    def get_pos_orient(prim):
        global_matrix = omni.usd.get_world_transform_matrix(prim)
        global_translate_pos = global_matrix.ExtractTranslation()
        global_translate_orient = global_matrix.ExtractRotation()
        local_translate_pos = omni.usd.get_local_transform_SRT(prim)                
        return (global_translate_pos, global_translate_orient, local_translate_pos)

    def publish_rgb_stream(camera: Camera, freq = 10):
        render_product = camera.render_product_path
        step_size = int(60/freq)
        topic_name = camera.name
        queue_size = 1
        node_namespace = ""
        frame_id = camera.prim_path.split("/")[-1] 
        rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
        writer = rep.writers.get(rv + "ROS2PublishImage")
        writer.initialize(
            frameId=frame_id,
            nodeNamespace=node_namespace,
            queueSize=queue_size,
            topicName=topic_name
        )
        writer.attach([render_product])
        gate_path = omni.syntheticdata.SyntheticData._get_node_path(
            rv + "IsaacSimulationGate", render_product
        )
        og.Controller.attribute(gate_path + ".inputs:step").set(step_size)
        return

    def publish_pointcloud_from_depth(camera: Camera, freq = 10):
        # The following code will link the camera's render product and publish the data to the specified topic name.
        render_product = camera.render_product_path
        step_size = int(60/freq)
        topic_name = camera.name+"_pointcloud" # Set topic name to the camera's name
        queue_size = 1
        node_namespace = ""
        frame_id = camera.prim_path.split("/")[-1] # This matches what the TF tree is publishing.
        # Note, this pointcloud publisher will convert the Depth image to a pointcloud using the Camera intrinsics.
        # This pointcloud generation method does not support semantic labeled objects.
        rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
            sd.SensorType.DistanceToImagePlane.name
        )
        writer = rep.writers.get(rv + "ROS2PublishPointCloud")
        writer.initialize(
            frameId=frame_id,
            nodeNamespace=node_namespace,
            queueSize=queue_size,
            topicName=topic_name
        )
        writer.attach([render_product])
        gate_path = omni.syntheticdata.SyntheticData._get_node_path(
            rv + "IsaacSimulationGate", render_product
        )
        og.Controller.attribute(gate_path + ".inputs:step").set(step_size)
        return
