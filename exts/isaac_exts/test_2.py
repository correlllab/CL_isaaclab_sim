# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import argparse
import sys

import carb
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.sensors.physx import RotatingLidarPhysX
from isaacsim.storage.native import get_assets_root_path

import isaacsim.core.utils.extensions as extensions_utils
extensions_utils.enable_extension("isaacsim.ros2.bridge")
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import std_msgs
from std_msgs.msg import Header
import sensor_msgs


class TestROS2PCPublisherNode(Node):
    def __init__(self):
        super().__init__("test_ros2_pc_publisher_node")
        self._publisher = self.create_publisher(PointCloud2, "/test_pc", 10)



    def publish_pc(self, points):
        try:
            ros_dtype = sensor_msgs.msg.PointField.FLOAT32
            dtype = np.float32
            itemsize = np.dtype(dtype).itemsize

            data = points.astype(dtype).tobytes()

            fields = [sensor_msgs.msg.PointField(name=n, offset=i*itemsize, datatype=ros_dtype, count=1) for i, n in enumerate('xyz')]
            header = std_msgs.msg.Header(frame_id="map", stamp=self.get_clock().now().to_msg())
            self._publisher.publish(sensor_msgs.msg.PointCloud2(
                header=header,
                height=1,
                width=points.shape[0],
                is_dense=False,
                is_bigendian=False,
                fields=fields,
                point_step=(itemsize * 3), # Every point consists of three float32s.
                row_step=(itemsize * 3 * points.shape[0]),
                data=data
            ))

            print(f"{points.shape[0]} points published")
        except:
            print(f"publishing failed")
            



parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()


my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()
asset_path = assets_root_path + "/Isaac/Robots/NVIDIA/Carter/carter_v1_physx_lidar.usd"
my_carter = my_world.scene.add(
    WheeledRobot(
        prim_path="/World/Carter",
        name="my_carter",
        wheel_dof_names=["left_wheel", "right_wheel"],
        create_robot=True,
        usd_path=asset_path,
        position=np.array([0, 0.0, 0.5]),
    )
)

#my_lidar = my_world.scene.add(
#    RotatingLidarPhysX(
#        prim_path="/World/Carter/chassis_link/lidar", name="lidar", translation=np.array([-0.06, 0, 2]), fov=(360.0, 60.0), valid_range=(0.4, 100), resolution=(0.2, 0.2), rotation_frequency=10.0
#    )
#)
import asyncio  # Used to run sample asynchronously to not block rendering thread

import omni  # Provides the core omniverse APIs
from isaacsim.sensors.physx import _range_sensor  # Imports the python bindings to interact with Lidar sensor
from pxr import Gf, Semantics, UsdGeom, UsdPhysics  # pxr usd imports used to create cube

stage = omni.usd.get_context().get_stage()  # Used to access Geometry
timeline = omni.timeline.get_timeline_interface()

lidarInterface = _range_sensor.acquire_lidar_sensor_interface()  # Used to interact with the LIDAR
lidarPath = "/LidarName"
# Create Lidar prim
result, prim = omni.kit.commands.execute(
    "RangeSensorCreateLidar",
    path=lidarPath,
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


cube_1 = my_world.scene.add(
    DynamicCuboid(prim_path="/World/cube", name="cube_1", position=np.array([2, 2, 2.5]), scale=np.array([20, 0.2, 5]))
)

cube_2 = my_world.scene.add(
    DynamicCuboid(
        prim_path="/World/cube_2", name="cube_2", position=np.array([2, -2, 2.5]), scale=np.array([20, 0.2, 5])
    )
)

my_controller = DifferentialController(name="simple_control", wheel_radius=0.24, wheel_base=0.56)

my_world.reset()
reset_needed = False
rclpy.init()
ros2_node = TestROS2PCPublisherNode() 

async def publish_lidar_data():
    await omni.kit.app.get_app().next_update_async()
    timeline.pause()
    lidar_data = lidarInterface.get_point_cloud_data("/World" + lidarPath)
    lidar_data = np.reshape(lidar_data, (-1, 3))
    ros2_node.publish_pc(lidar_data)
    timeline.play()

from omni.kit.async_engine import run_coroutine

while simulation_app.is_running():
    my_world.step(render=True)

    run_coroutine(publish_lidar_data())
    if my_world.is_stopped() and not reset_needed:

        reset_needed = True
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            my_controller.reset()
            reset_needed = False
        # print(imu_sensor.get_current_frame())
            # print(my_lidar.get_current_frame())
            # forward
        my_carter.apply_wheel_actions(my_controller.forward(command=[0.05, 0]))
            # rotate
    if args.test is True:

        break
simulation_app.close()
