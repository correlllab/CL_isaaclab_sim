from sensor_msgs.msg import CompressedImage, CameraInfo
import rclpy
from rclpy.node import Node
from cl.ros2.realsense.ogn.OgnClRos2RealsensePyDatabase import OgnClRos2RealsensePyDatabase

from isaacsim.ros2.bridge import read_camera_info
#if this import fails, then ros2 bridge needs to be enabled
from isaacsim.sensors.camera import Camera
import numpy as np
from isaacsim.core.nodes import BaseResetNode
from nvjpeg import NvJpeg
class OgnClRos2RealsensePyInternalState(BaseResetNode):
    """Convenience class for maintaining per-node state information"""

    def __init__(self):
        """Instantiate the per-node state information"""
        self._latest_msg = None
        self._ros2_node = None
        self._compressed_jpeg_publisher = None
        self._camera_info_publisher = None 
        self._camera = None
        self._nvjpeg = None
        super().__init__(initialize=False)

#    def create_cameras(self):
#        path_names = {"l_depth" : "/World/envs/env_0/Robot/h1_2_26dof_with_inspire_rev_1_0_with_CL_realsense/L_hand_base_link/CL_L_realsense/rsd455/RSD455/Camera_Pseudo_Depth", "l_rgb" : "/World/envs/env_0/Robot/h1_2_26dof_with_inspire_rev_1_0_with_CL_realsense/L_hand_base_link/CL_L_realsense/rsd455/RSD455/Camera_OmniVision_OV9782_Color", "r_depth" : "/World/envs/env_0/Robot/h1_2_26dof_with_inspire_rev_1_0_with_CL_realsense/R_hand_base_link/CL_R_realsense/rsd455/RSD455/Camera_Pseudo_Depth", "r_rgb"  : "/World/envs/env_0/Robot/h1_2_26dof_with_inspire_rev_1_0_with_CL_realsense/R_hand_base_link/CL_R_realsense/rsd455/RSD455/Camera_OmniVision_OV9782_Color"}
#
#        for (name, path) in path_names.items():
#            camera = Camera(path, name)
#            camera.resolution = (1280, 720)
#            camera.initialize()
#            if "Depth" in path:
#                camera.add_pointcloud_to_frame()
#            elif "Color" in path:
#                camera.add_rgb_to_frame()
#            else:
#                print("no annotator added")
        
    @property
    def latest_rgb_data_jpeg_format(self, jpeg_accuracy=70):
        return self._nvjpeg.encode(self._camera.get_rgb(), jpeg_accuracy)

    def initialize(self, node_name, topic_name):
        try:
            rclpy.init()
        except:
            pass
        if not self._ros2_node:
            self._ros2_node = rclpy.create_node(node_name=node_name)

        
        if not self._compressed_jpeg_publisher:
            self._compressed_jpeg_publisher = self._ros2_node.create_publisher(msg_type=CompressedImage, topic=topic_name, qos_profile=10)

        if not self._camera_info_publisher:
            self._camera_info_publisher = self._ros2_node.create_publisher(msg_typ)
        if not self._nvjpeg:
            self._nvjpeg = NvJpeg()

        self.initialized = True


        self._camera = Camera("/World/envs/env_0/Robot/h1_2_26dof_with_inspire_rev_1_0_with_CL_realsense/L_hand_base_link/CL_L_realsense/rsd455/RSD455/Camera_OmniVision_OV9782_Color", "l_rgb")
    
        self._camera.initialize()
        self._camera.add_rgb_to_frame()
    def publish_msg(self, timeout_sec=0.01):
        self._compressed_jpeg_publisher.publish(self._latest_msg)

    def wipe_reset(self):
        if self._ros2_node:
            self._ros2_node.destroy_publisher(self._compressed_jpeg_publisher)
            self._ros2_node.destroy_node()

        self._latest_msg = None
        self._ros2_node = None
        self._compressed_jpeg_publisher = None
        self._camera = None
        self._nvjpeg = None
        self.initialized = False
        rclpy.try_shutdown()

class OgnClRos2RealsensePy:
    """The Ogn node class"""

    @staticmethod
    def internal_state():
        """Returns an object that contains per-node state information"""
        return OgnClRos2RealsensePyInternalState()

    @staticmethod
    def compute(db) -> bool:
        """Compute the output based on inputs and internal state"""
        state = db.per_instance_state
        try:
            if not state.initialized:
                state.initialize(node_name="cl_ros2_realsense_node", topic_name="/l_rgb/color/image_raw/compressed")

            msg = CompressedImage()
            msg.header.stamp = state._ros2_node.get_clock().now().to_msg()

            msg.header.frame_id = "l_rgb"
            msg.format = "jpeg"
            msg.data = state.latest_rgb_data_jpeg_format
            state._latest_msg = msg
            state.publish_msg()
            # -----------------
            # read input values
            # do custom computation
            # write output values
            # -----------------
        except Exception as e:
            db.log_error(f"Computation error: {e}")
            return False
        return True


    @staticmethod
    def release(node):
        try:
            state = OgnClRos2RealsensePyDatabase.per_instance_internal_state(node)
        except Exception as e:
            return
        state.wipe_reset()
        state.initialized = False
