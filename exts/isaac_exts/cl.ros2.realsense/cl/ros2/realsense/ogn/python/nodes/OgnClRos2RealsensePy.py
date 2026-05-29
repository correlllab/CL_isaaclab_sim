from sensor_msgs.msg import CompressedImage, CameraInfo
import rclpy
from rclpy.node import Node
from cl.ros2.realsense.ogn.OgnClRos2RealsensePyDatabase import OgnClRos2RealsensePyDatabase
import json

from isaacsim.ros2.bridge import read_camera_info
#if this import fails, then ros2 bridge needs to be enabled
from isaacsim.sensors.camera import Camera, SingleViewDepthSensor
import numpy as np
from isaacsim.core.nodes import BaseResetNode
from nvjpeg import NvJpeg
class OgnClRos2RealsensePyInternalState(BaseResetNode):
    """Convenience class for maintaining per-node state information"""

    def __init__(self):
        """Instantiate the per-node state information"""
        self._ros2_node = None
        self._jpeg_publishers = None
        self._cameras = None
        self._nvjpeg = None
        super().__init__(initialize=False)

    def initialize(self, node_name, json_config_path):
        try:
            rclpy.init()
        except:
            pass
        if not self._ros2_node:
            self._ros2_node = rclpy.create_node(node_name=node_name)

        f = open(json_config_path)
        camera_config = json.load(f)

        if not self._cameras:
            self._cameras = []

            for cam_name in camera_config['cameras']:
                if "rgb" in cam_name:
                    cam = Camera(prim_path=camera_config['cameras'][cam_name]['path'], name=cam_name, resolution=(1280, 720))
                    cam.initialize()
                    cam.add_rgb_to_frame()
                elif "depth" in cam_name:
                    cam = SingleViewDepthSensor(prim_path=camera_config['cameras'][cam_name]['path'], name=cam_name, resolution=(1280, 720))
                    cam.initialize(attach_rgb_annotator=False)
                    cam.attach_annotator("DepthSensorDistance")
                    cam.attach_annotator("distance_to_image_plane")
                else:
                    print("no-op")

                self._cameras.append(cam)

        if not self._jpeg_publishers:
            self._jpeg_publishers = {}

            for cam_name in camera_config['cameras']:
                topic_name = "/"
                topic_name += cam_name
                if "rgb" in cam_name:
                    topic_name += "/color/image_raw/compressed"
                elif "depth" in cam_name:
                    topic_name += "/depth/image_raw/compressed"
                else:
                    print("something weird happened with the topic names")
                pub = self._ros2_node.create_publisher(msg_type=CompressedImage, topic=topic_name, qos_profile=10)
                self._jpeg_publishers[cam_name] = pub

        if not self._nvjpeg:
            self._nvjpeg = NvJpeg()

        self.initialized = True

    def get_latest_cam_data_as_jpeg(self, target_cam_name, jpeg_accuracy=70):
        cam_obj = [cam for cam in self._cameras if cam.name == target_cam_name][0]
        if "rgb" in target_cam_name:
            raw_data = cam_obj.get_rgb()
        elif "depth" in target_cam_name:
            raw_grey_data = cam_obj.get_current_frame()["distance_to_image_plane"]
            raw_data = np.stack([raw_grey_data, np.zeros(shape=(720, 1280), dtype=raw_grey_data.dtype), np.zeros(shape=(720,1280), dtype=raw_grey_data.dtype)], axis=-1)
        else:
            print("something weird happened")
        try:
            return self._nvjpeg.encode(raw_data, jpeg_accuracy)
        except Exception as e:
            print(e)
            return []

    def batch_wipe_reset(self):
        if self._ros2_node:
            self._ros2_node.destroy_publisher(self._compressed_jpeg_publisher)
            self._ros2_node.destroy_node()

        self._ros2_node = None
        self._jpeg_publishers = None
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
                state.initialize(node_name="cl_ros2_realsense_node", json_config_path="/home/code/CL_isaaclab_sim/exts/isaac_exts/cl.ros2.realsense/cl/ros2/realsense/ogn/python/nodes/rs_config.json")

            msg_batch = []
            for cam in state._cameras:
                msg = CompressedImage()
                msg.header.stamp = state._ros2_node.get_clock().now().to_msg()
                msg.header.frame_id = cam.name
                msg.format = "jpeg"
                msg.data = state.get_latest_cam_data_as_jpeg(cam.name)
                state._jpeg_publishers[cam.name].publish(msg)

           # msg.header.frame_id = "l_rgb"
           # msg.format = "jpeg"
           # msg.data = state.latest_rgb_data_jpeg_format
           # state._latest_msg = msg
           # state.publish_msg()
            # -----------------
            # read input values
            # do custom computation
            # write output values
            # -----------------
        except Exception as e:
            import traceback
            traceback.print_exc()
            db.log_error(f"Computation error: {e}")
            return False
        return True


    @staticmethod
    def release(node):
        try:
            state = OgnClRos2RealsensePyDatabase.per_instance_internal_state(node)
        except Exception as e:
            return
        state.batch_wipe_reset()
        state.initialized = False





















































































#from sensor_msgs.msg import CompressedImage, CameraInfo
#import rclpy
#from rclpy.node import Node
#from cl.ros2.realsense.ogn.OgnClRos2RealsensePyDatabase import OgnClRos2RealsensePyDatabase
#
#from isaacsim.ros2.bridge import read_camera_info
##if this import fails, then ros2 bridge needs to be enabled
#from isaacsim.sensors.camera import Camera
#import numpy as np
#from isaacsim.core.nodes import BaseResetNode
#from nvjpeg import NvJpeg
#class OgnClRos2RealsensePyInternalState(BaseResetNode):
#    """Convenience class for maintaining per-node state information"""
#
#    def __init__(self):
#        """Instantiate the per-node state information"""
#        self._latest_msg = None
#        self._ros2_node = None
#        self._compressed_jpeg_publisher = None
#        self._camera_info_publisher = None 
#        self._camera = None
#        self._nvjpeg = None
#        super().__init__(initialize=False)
#
#        
#    @property
#    def latest_rgb_data_jpeg_format(self, jpeg_accuracy=70):
#        return self._nvjpeg.encode(self._camera.get_rgb(), jpeg_accuracy)
#
#    def initialize(self, node_name, topic_name):
#        try:
#            rclpy.init()
#        except:
#            pass
#        if not self._ros2_node:
#            self._ros2_node = rclpy.create_node(node_name=node_name)
#
#        
#        if not self._compressed_jpeg_publisher:
#            self._compressed_jpeg_publisher = self._ros2_node.create_publisher(msg_type=CompressedImage, topic=topic_name, qos_profile=10)
#
#        #if not self._camera_info_publisher:
#        #    self._camera_info_publisher = self._ros2_node.create_publisher(msg_typ)
#        if not self._nvjpeg:
#            self._nvjpeg = NvJpeg()
#
#        self.initialized = True
#
#
#        self._camera = Camera("/World/envs/env_0/Robot/h1_2_26dof_with_inspire_rev_1_0_with_CL_realsense/L_hand_base_link/CL_L_realsense/rsd455/RSD455/Camera_OmniVision_OV9782_Color", "l_rgb")
#    
#        self._camera.initialize()
#        self._camera.add_rgb_to_frame()
#    def publish_msg(self, timeout_sec=0.01):
#        self._compressed_jpeg_publisher.publish(self._latest_msg)
#
#    def wipe_reset(self):
#        if self._ros2_node:
#            self._ros2_node.destroy_publisher(self._compressed_jpeg_publisher)
#            self._ros2_node.destroy_node()
#
#        self._latest_msg = None
#        self._ros2_node = None
#        self._compressed_jpeg_publisher = None
#        self._camera = None
#        self._nvjpeg = None
#        self.initialized = False
#        rclpy.try_shutdown()
#
#class OgnClRos2RealsensePy:
#    """The Ogn node class"""
#
#    @staticmethod
#    def internal_state():
#        """Returns an object that contains per-node state information"""
#        return OgnClRos2RealsensePyInternalState()
#
#    @staticmethod
#    def compute(db) -> bool:
#        """Compute the output based on inputs and internal state"""
#        state = db.per_instance_state
#        try:
#            if not state.initialized:
#                state.initialize(node_name="cl_ros2_realsense_node", topic_name="/l_rgb/color/image_raw/compressed")
#
#            msg = CompressedImage()
#            msg.header.stamp = state._ros2_node.get_clock().now().to_msg()
#
#            msg.header.frame_id = "l_rgb"
#            msg.format = "jpeg"
#            msg.data = state.latest_rgb_data_jpeg_format
#            state._latest_msg = msg
#            state.publish_msg()
#            # -----------------
#            # read input values
#            # do custom computation
#            # write output values
#            # -----------------
#        except Exception as e:
#            db.log_error(f"Computation error: {e}")
#            return False
#        return True
#
#
#    @staticmethod
#    def release(node):
#        try:
#            state = OgnClRos2RealsensePyDatabase.per_instance_internal_state(node)
#        except Exception as e:
#            return
#        state.wipe_reset()
#        state.initialized = False
