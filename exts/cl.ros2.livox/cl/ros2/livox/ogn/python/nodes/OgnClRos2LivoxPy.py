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

from sensor_msgs.msg import PointCloud2, PointField, Imu
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Header
import rclpy
from rclpy.node import Node
from omni.kit.async_engine import run_coroutine
from isaacsim.core.nodes import BaseResetNode
from isaacsim.sensors.physx import _range_sensor
from isaacsim.sensors.physics import _sensor
import isaacsim.core.utils.stage as stage_utils
from pxr import Gf, UsdPhysics, PhysxSchema
import omni.usd
import omni.kit.commands

import numpy as np
import omni.kit
import traceback
import asyncio

class OgnClRos2LivoxPyInternalState(BaseResetNode):
    """Convenience class for maintaining per-node state information"""

    def __init__(self):
        """Instantiate the per-node state information"""
        self._ros2_node = None
        self._lidar_publisher = None
        self._lidar_interface = None
        self._lidar_prim = None

        self._imu_prim = None
        self._imu_interface = None
        self._imu_publisher = None
        self._timeline = None

        status = False
        super().__init__(initialize=False)

    def initialize(self):
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
            path="/livox_lidar",
            parent="/World/envs/env_0/Robot/lidar_link",
            min_range=0.4,
            max_range=40.0,
            draw_points=False,
            draw_lines=False,
            horizontal_fov=360.0,
            vertical_fov=60.0,
            horizontal_resolution=0.272,
            vertical_resolution=0.4,
            rotation_rate=0.0,
            high_lod=True,
            yaw_offset=0.0,
            enable_semantics=True,
        )


        if not self._lidar_publisher:
            self._lidar_publisher = self._ros2_node.create_publisher(msg_type=PointCloud2, topic="/test_pc", qos_profile=10)

        if not self._imu_interface:
            self._imu_interface = _sensor.acquire_imu_sensor_interface()

        if not self._imu_prim:
            #prim = omni.usd.get_context().get_stage().GetPrimAtPath("/World/envs/env_0/Robot/h1_2_26dof_with_inspire_rev_1_0/imu_link")
            #if prim is not None:
            #    UsdPhysics.RigidBodyAPI.Apply(prim)

#            stage = omni.usd.get_context().get_stage()
#            try:
#                imu_link_prim = stage.GetPrimAtPath("/World/envs/env_0/Robot/h1_2_26dof_with_inspire_rev_1_0/imu_link")
#                UsdPhysics.RigidBodyAPI.Apply(imu_link_prim)
#                UsdPhysics.CollisionAPI.Apply(imu_link_prim)
#                UsdPhysics.MassAPI.Apply(imu_link_prim)
#                UsdPhysics.MassAPI(imu_link_prim).CreateMassAttr(0.1)
#                rb = UsdPhysics.RigidBodyAPI(imu_link_prim)
#                rb.CreateRigidBodyEnabledAttr(True)
#                rb.CreateKinematicEnabledAttr(False)
#
#            except:
#                import traceback
#                traceback.print_exc()



            pass
            #result, self._imu_prim = omni.kit.commands.execute(
            #    "IsaacSensorCreateImuSensor",
            #    path="livox_imu",
            #    parent="/World/envs/env_0/Robot/h1_2_26dof_with_inspire_rev_1_0/imu_link",
            #    sensor_period=1,
            #    linear_acceleration_filter_size=10,
            #    angular_velocity_filter_size=10,
            #    orientation_filter_size=10,
            #    translation = Gf.Vec3d(0, 0, 1),
            #    orientation = Gf.Quatd(1, 0, 0, 0),
            #)
#            prim = omni.usd.get_context().get_stage().GetPrimAtPath("/World/envs/env_0/Robot/h1_2_26dof_with_inspire_rev_1_0/imu_link")
#            UsdPhysics.RigidBodyAPI.Apply(prim)

#            self._imu_prim = UsdPhysics.RigidBodyAPI.Apply(self._imu_prim)
            #self._imu_prim = PhysxSchema.AnotherPhysxAPI.Apply(self._imu_prim)




        if not self._imu_publisher:
            self._imu_publisher = self._ros2_node.create_publisher(msg_type=Imu, topic="/test_imu", qos_profile=10)

        if not self._timeline:
            self._timeline = omni.timeline.get_timeline_interface()

        if not self._timeline:
            breakpoint()


        self.initialized = True

    async def publish_livox_lidar_data(self):
        try:
            await omni.kit.app.get_app().next_update_async()
            await stage_utils.update_stage_async()
            self._timeline.pause()
            lidar_data = self._lidar_interface.get_point_cloud_data("/World/envs/env_0/Robot/lidar_link/livox_lidar")

            lidar_data = np.reshape(lidar_data, (-1, 3))
            ros_dtype = PointField.FLOAT32
            dtype = np.float32
            itemsize = np.dtype(dtype).itemsize

            data = lidar_data.astype(dtype).tobytes()
            fields = [PointField(name=n, offset=i*itemsize, datatype=ros_dtype, count=1) for i, n in enumerate('xyz')]

            header = Header(frame_id="lidar_link", stamp=self._ros2_node.get_clock().now().to_msg())

            self._lidar_publisher.publish(PointCloud2(
                header=header,
                height=1,
                width=lidar_data.shape[0],
                is_dense=False,
                is_bigendian=False,
                fields=fields,
                point_step=(itemsize * 3), 
                row_step=(itemsize * 3 * lidar_data.shape[0]),
                data=data
            ))

            self._timeline.play()
            print(f"published {lidar_data.shape[0]} points")
            print(lidar_data.shape)
        except:
            print(f"publishing lidar failed")
            print(lidar_data.shape)
            traceback.print_exc()

    async def publish_livox_imu_data(self, quat_w, pos_w, lin_vel_b, lin_acc_b, ang_vel_b, ang_acc_b):
        try:
            await omni.kit.app.get_app().next_update_async()
            await stage_utils.update_stage_async()
            self._timeline.pause()
            print(f"{quat_w=}")
            print(f"{pos_w=}")
            print(f"{lin_vel_b}")
            print(f"{lin_acc_b}")
            print(f"{ang_vel_b}")
            quat_w = np.float32(quat_w)
            np.float32(pos_w)
            #imu_data = self._imu_interface.get_sensor_reading("/World/envs/env_0/Robot/h1_2_26dof_with_inspire_rev_1_0/imu_link/livox_imu", use_latest_data=True, read_gravity=True)
            orientation_as_quat = Quaternion(x=float(quat_w[0]), y=float(quat_w[1]), z=float(quat_w[2]), w=float(quat_w[3]))
            orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            angular_velocity = Vector3(x=float(ang_vel_b[0]), y=float(ang_vel_b[1]), z=float(ang_vel_b[2]))
            angular_velocity_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            linear_acceleration = Vector3(x=float(lin_acc_b[0]), y=float(lin_acc_b[1]), z=float(lin_acc_b[2]))
            linear_acceleration_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


            header = Header(frame_id="imu_link", stamp=self._ros2_node.get_clock().now().to_msg())

            self._imu_publisher.publish(Imu(
                header=header,
                orientation=orientation_as_quat,
                orientation_covariance=orientation_covariance,
                angular_velocity=angular_velocity,
                angular_velocity_covariance=angular_velocity_covariance,
                linear_acceleration=linear_acceleration,
                linear_acceleration_covariance=linear_acceleration_covariance
            ))
            
            self._timeline.play()
            print(f"published: {orientation_as_quat}\n, {orientation_covariance}\n, {angular_velocity}\n, {angular_velocity_covariance}\n, {linear_acceleration}\n, {linear_acceleration_covariance}\n")
        except:
            print("publishing imu data failed")
            traceback.print_exc()
            

    def batch_wipe_reset(self):
        if self._ros2_node:
            self._ros2_node.destroy_publisher(self._lidar_publisher)
            self._ros2_node.destroy_node()
        
        self._lidar_publisher = None
        self._lidar_interface = None
        self._lidar_prim = None

        self._imu_prim = None
        self._imu_interface = None
        self._imu_publisher = None
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
        if not state.initialized:
            state.initialize()

        #try:
        #    run_coroutine(state.publish_livox_lidar_data())
        #except Exception as e:
        #    breakpoint()
        #    db.log_error(f"Computation error: {e}")
        #    return False

        try:
            #run_coroutine(state.publish_livox_imu_data(db.inputs.quat_w, db.inputs.pos_w, db.inputs.lin_vel_b, db.inputs.lin_acc_b, db.inputs.ang_vel_b, db.inputs.ang_acc_b))
            run_coroutine(state.publish_livox_lidar_data())
        except Exception as e:
            breakpoint()
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
