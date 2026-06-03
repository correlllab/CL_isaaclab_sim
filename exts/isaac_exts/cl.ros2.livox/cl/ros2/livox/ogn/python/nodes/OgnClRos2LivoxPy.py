"""
OmniGraph core Python API:
  https://docs.omniverse.nvidia.com/kit/docs/omni.graph/latest/Overview.html

OmniGraph attribute data types:
  https://docs.omniverse.nvidia.com/kit/docs/omni.graph.docs/latest/dev/ogn/attribute_types.html

Collection of OmniGraph code examples in Python:
  https://docs.omniverse.nvidia.com/kit/docs/omni.graph.docs/latest/dev/ogn/ogn_code_samples_python.html

Collection of OmniGraph tutorials:
  https://docs.omniverse.nvidia.com/kit/docs/omni.graph.tutorials/latest/Overview.html
"""

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import rclpy
from rclpy.node import Node
from omni.kit.async_engine import run_coroutine
from isaacsim.core.nodes import BaseResetNode

import omni.kit

class OgnClRos2LivoxPyInternalState(BaseResetNode):
    """Convenience class for maintaining per-node state information"""

    def __init__(self):
        """Instantiate the per-node state information"""
        self._ros2_node = None
        self._lidar_publisher = None
        self._lidar_interface = None
        self._lidar_prim = None
        self._timeline = None

        status = False
        super().__init__(initialize=False)

    def initialize(self):
        breakpoint()
        try:
            rclpy.init()
        except:
            pass

        if not self._ros2_node:
            self._ros2_node = rclpy.create_node("test_pc_livox")

        if not self._lidar_interface:
            self._lidar_interface = _range_sensor.acquire_lidar_sensor_interface()

        if not self._lidar_prim:
            result, self._lidar_prim = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path="/Lidar",
            parent="/World",
            min_range=0.4,
            max_range=100.0,
            draw_points=True,
            draw_lines=False,
            horizontal_fov=360.0,
            vertical_fov=60.0,
            horizontal_resolution=0.4,
            vertical_resolution=0.4,
            rotation_rate=0.0,
            high_lod=True,
            yaw_offset=0.0,
            enable_semantics=True,
        )

        if not self._lidar_publisher:
            self._lidar_publisher = self._ros2_node.create_publisher(msg_type=PointCloud2, topic="/test_pc", qos_profile=10)

        if not self._timeline:
            self._timeline = omni.timeline.get_timeline_interface()
        self.initialized = True

    async def publish_livox_lidar_data(self):
        breakpoint()
        try:
            await omni.kit.app.get_app().next_update_async()
            self._timeline.pause()
            lidar_data = self._lidar_interface.get_point_cloud_data("/World/Lidar")
            ros_dtype = PointField.FLOAT32
            dtype = np.float32
            itemsize = np.dtype(dtype).itemsize

            data = lidar_data.astype(dtype).tobytes()
            fields = [PointField(name=n, offset=i*itemsize, datatype=ros_dtype, count=1) for i, n in enumerate('xyz')]

            header = Header(frame_id="map", stamp=self.get_clock().now().to_msg())

            self._lidar_publisher.publish(PointCloud2(
                header=header,
                height=1,
                width=lidar_data.shape[0],
                is_dense=False,
                is_bigendian=False,
                fields=fields,
                point_step=(itemsize * 3), # Every point consists of three float32s.
                row_step=(itemsize * 3 * lidar_data.shape[0]),
                data=data
            ))
            
            self._timeline.play()
            print(f"published {lidar_data.shape[0]} points")
        except:
            print(f"publishing failed")

    def batch_wipe_reset(self):
        if self._ros2_node:
            self._ros2_node.destroy_publisher(self._lidar_publisher)
            self._ros2_node.destroy_node()
        
        self._lidar_publisher = None
        self._lidar_interface = None
        self._lidar_prim = None
        self._timeline = None
        self.initialized = False
        rclpy.try_shutdown()

            
        

class OgnClRos2LivoxPy:
    """The Ogn node class"""

    @staticmethod
    def internal_state():
        """Returns an object that contains per-node state information"""
        return OgnClRos2LivoxPyInternalState()

    @staticmethod
    def compute(db) -> bool:
        """Compute the output based on inputs and internal state"""
        state = db.per_instance_state

        try:
            run_coroutine(state.publish_livox_lidar_data())
        except Exception as e:
            db.log_error(f"Computation error: {e}")
            return False
        return True

    @staticmethod
    def release(node):
        try:
            state = OgnClRos2LivoxPyDatabase.per_instance_internal_state(node)
        except Exception as e:
            return
        state.batch_wipe_reset()
        state.initialized = False
