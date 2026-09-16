"""ROS 2 transport for Isaac Lab camera, ray-caster and IMU samples.

The ray-caster uses a uniform angular approximation of a MID360 scan.
Livox CustomMsg uses the same CycloneDDS transport as the Unitree bridge.
Only measured sensor channels are exposed, with no world poses or pose TF.
"""
from array import array
import io
import math
import os
from pathlib import Path
import struct
import sys

import numpy as np
from PIL import Image as PILImage


POINT_DTYPE = np.dtype([
    ('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'),
    ('tag', 'u1'), ('line', 'u1'), ('timestamp', '<f8'),
])


def lidar_payload(hits, position, quaternion, horizontal_samples, period):
    hits = np.asarray(hits)
    valid = np.isfinite(hits).all(axis=-1)
    points = points_in_sensor_frame(hits, position, quaternion)
    indices = np.flatnonzero(valid)
    result = np.zeros(len(points), dtype=POINT_DTYPE)
    for i, axis in enumerate('xyz'):
        result[axis] = points[:, i]
    result['line'] = (indices // horizontal_samples) % 6
    # Like RoboCasa, acquisition offsets approximate an azimuth sweep. The
    # ray-caster itself samples all rays at once, so motion distortion is absent.
    result['timestamp'] = ((indices % horizontal_samples) / horizontal_samples * period * 1e9).astype(np.uint32)
    return result


CAMERAS = {
    "front_camera": ("head", "camera_color_optical_frame"),
    "left_wrist_camera": ("left_hand", "left_hand_camera_color_optical_frame"),
    "right_wrist_camera": ("right_hand", "right_hand_camera_color_optical_frame"),
}


def stamp_parts(seconds):
    if not math.isfinite(seconds) or seconds < 0:
        raise ValueError("simulation time must be finite and nonnegative")
    return divmod(round(seconds * 1_000_000_000), 1_000_000_000)


def as_numpy(value):
    if hasattr(value, "detach"):
        return value.detach().cpu().numpy()
    return np.asarray(value)


def depth_millimeters(depth):
    depth = np.asarray(depth)
    valid = np.isfinite(depth) & (depth > 0)
    return np.clip(np.where(valid, depth, 0) * 1000, 0, 65535).astype('<u2')


def encode_depth(depth_mm):
    stream = io.BytesIO()
    PILImage.fromarray(depth_mm).save(stream, format="PNG")
    return struct.pack('<Iff', 0, 0., 0.) + stream.getvalue()


def points_in_sensor_frame(hits, position, quaternion):
    points = np.asarray(hits)
    points = points[np.isfinite(points).all(axis=-1)] - np.asarray(position)
    q = np.asarray(quaternion, dtype=np.float64)
    q /= np.linalg.norm(q)
    w, x, y, z = q
    rotation = np.array([
        [1 - 2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1 - 2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1 - 2*(x*x+y*y)],
    ])
    return np.ascontiguousarray(points @ rotation, dtype='<f4')


def bootstrap_ros():
    """Locate Isaac's Python-ABI-matched ROS packages.

    launch_isaac.sh puts the SDK Cyclone library before bundled ROS libraries
    in LD_LIBRARY_PATH, so Python DDS and ROS RMW share one native library.
    This must happen before interpreter startup; changing it here cannot fix dlopen.
    """
    try:
        import rclpy
        return rclpy
    except ModuleNotFoundError as exc:
        if exc.name != "rclpy":
            raise
    ros_dir = Path(os.environ.get("ISAAC_ROS_PYTHON_PATH", "/isaac-sim/exts/isaacsim.ros2.bridge/humble/rclpy"))
    if not ros_dir.is_dir():
        raise RuntimeError("ROS 2 Python packages unavailable; launch through docker/scripts/launch_isaac.sh")
    sys.path.insert(0, str(ros_dir))
    import rclpy
    return rclpy


class RosSensorBridge:
    """Single-environment, synchronous publisher; call after env.step()."""

    def __init__(self, scene):
        domain = int(os.environ.get("ROS_DOMAIN_ID", "1") or "1")
        if not 1 <= domain <= 232:
            raise ValueError("sensor bridge requires a simulation ROS_DOMAIN_ID in 1..232")
        os.environ["ROS_DOMAIN_ID"] = str(domain)
        self.rclpy = bootstrap_ros()
        from rclpy.context import Context
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image, CompressedImage, CameraInfo, Imu, PointCloud2, PointField
        from rosgraph_msgs.msg import Clock
        from builtin_interfaces.msg import Time
        self.types = (Image, CompressedImage, CameraInfo, Imu, PointCloud2, PointField, Clock, Time)
        self.context = Context()
        self.node = None
        self.scene = scene
        self._last = {}
        self._publishers = {}
        self._livox = None
        self._camera_names = [name for name in CAMERAS if name in scene.sensors]
        try:
            self.rclpy.init(context=self.context)
            self.node = self.rclpy.create_node("isaac_sensor_bridge", context=self.context)
            def add(key, msg_type, topic, qos=qos_profile_sensor_data):
                self._publishers[key] = self.node.create_publisher(msg_type, topic, qos)
            add("clock", Clock, "/clock", 10)
            for name in self._camera_names:
                namespace, _ = CAMERAS[name]
                root = f"/realsense/{namespace}"
                for key, msg_type, suffix in (
                    ("rgb", Image, "color/image_raw"),
                    ("jpeg", CompressedImage, "color/image_raw/compressed"),
                    ("depth", Image, "aligned_depth_to_color/image_raw"),
                    ("png", CompressedImage, "aligned_depth_to_color/image_raw/compressedDepth"),
                    ("info", CameraInfo, "color/camera_info"),
                ):
                    add((name, key), msg_type, f"{root}/{suffix}")
            if "lidar" in scene.sensors:
                from .livox_dds import LivoxPublisher
                self._livox = LivoxPublisher(domain)
                add("lidar", PointCloud2, "/livox/pointcloud")
                add("lidar_alias", PointCloud2, "/lidar/first_link")
                self.node.get_logger().info(
                    "Publishing MID360 angular approximation on /livox/pointcloud; "
                    "CycloneDDS supplies /livox/lidar CustomMsg.")
            if "imu" in scene.sensors:
                add("imu", Imu, "/livox/imu")
                add("imu_alias", Imu, "/imu/base_link")
        except Exception:
            self.close()
            raise

    def _due(self, name, seconds):
        previous = self._last.get(name)
        period = self.scene.sensors[name].cfg.update_period
        if previous is not None and seconds >= previous and seconds - previous < period - 1e-9:
            return False
        self._last[name] = seconds
        return True

    def publish(self, seconds):
        Image, CompressedImage, CameraInfo, Imu, PointCloud2, PointField, Clock, Time = self.types
        sec, nanosec = stamp_parts(seconds)
        stamp = Time(sec=sec, nanosec=nanosec)
        self._publishers['clock'].publish(Clock(clock=stamp))
        for name in self._camera_names:
            if self._due(name, seconds):
                self._publish_camera(name, stamp)
        if 'lidar' in self.scene.sensors and self._due('lidar', seconds):
            data = self.scene.sensors['lidar'].data
            cfg = self.scene.sensors['lidar'].cfg
            pattern = cfg.pattern_cfg
            samples = math.ceil((pattern.horizontal_fov_range[1] - pattern.horizontal_fov_range[0]) / pattern.horizontal_res)
            points = lidar_payload(as_numpy(data.ray_hits_w[0]), as_numpy(data.pos_w[0]), as_numpy(data.quat_w[0]), samples, cfg.update_period)
            msg = PointCloud2()
            msg.header.stamp, msg.header.frame_id = stamp, "lidar_link"
            msg.height, msg.width = 1, len(points)
            msg.fields = [PointField(name=name, offset=POINT_DTYPE.fields[name][1], datatype=kind, count=1)
                          for name, kind in [('x', 7), ('y', 7), ('z', 7), ('intensity', 7), ('tag', 2), ('line', 2), ('timestamp', 8)]]
            msg.is_bigendian, msg.is_dense = False, True
            msg.point_step, msg.row_step = POINT_DTYPE.itemsize, POINT_DTYPE.itemsize*len(points)
            msg.data = array('B', points.tobytes())
            self._livox.publish(sec, nanosec, points)
            self._publishers['lidar'].publish(msg)
            self._publishers['lidar_alias'].publish(msg)
        if 'imu' in self.scene.sensors and self._due('imu', seconds):
            data = self.scene.sensors['imu'].data
            msg = Imu()
            msg.header.stamp, msg.header.frame_id = stamp, "lidar_link"
            # The hardware Livox IMU has no orientation estimate.
            msg.orientation_covariance[0] = -1.
            for target, values in ((msg.angular_velocity, data.ang_vel_b[0]), (msg.linear_acceleration, data.lin_acc_b[0])):
                target.x, target.y, target.z = map(float, as_numpy(values))
            self._publishers['imu'].publish(msg)
            self._publishers['imu_alias'].publish(msg)

    def _publish_camera(self, name, stamp):
        Image, CompressedImage, CameraInfo, *_ = self.types
        sensor = self.scene.sensors[name]
        data = sensor.data
        frame = CAMERAS[name][1]
        rgb = np.ascontiguousarray(as_numpy(data.output['rgb'][0])[..., :3], dtype=np.uint8)
        h, w = rgb.shape[:2]

        def raw(key, pixels, encoding, step):
            msg = Image()
            msg.header.stamp, msg.header.frame_id = stamp, frame
            msg.height, msg.width, msg.encoding = h, w, encoding
            msg.is_bigendian, msg.step = 0, step
            msg.data = array('B', pixels.tobytes())
            self._publishers[(name, key)].publish(msg)

        def compressed(key, payload, format):
            msg = CompressedImage()
            msg.header.stamp, msg.header.frame_id = stamp, frame
            msg.format, msg.data = format, array('B', payload)
            self._publishers[(name, key)].publish(msg)

        raw('rgb', rgb, 'rgb8', w*3)
        stream = io.BytesIO()
        PILImage.fromarray(rgb).save(stream, format='JPEG', quality=80)
        compressed('jpeg', stream.getvalue(), 'rgb8; jpeg compressed bgr8')
        depth = as_numpy(data.output['distance_to_image_plane'][0]).reshape(h, w)
        # A clipped far-plane value is a missing return, as on a RealSense.
        depth = np.where(depth >= sensor.cfg.spawn.clipping_range[1]*.999, 0, depth)
        mm = depth_millimeters(depth)
        raw('depth', mm, '16UC1', w*2)
        compressed('png', encode_depth(mm), '16UC1; compressedDepth png')
        info = CameraInfo()
        info.header.stamp, info.header.frame_id = stamp, frame
        info.height, info.width = h, w
        info.distortion_model, info.d = 'plumb_bob', [0.]*5
        matrix = as_numpy(data.intrinsic_matrices[0]).astype(float)
        info.k = matrix.reshape(-1).tolist()
        info.r = np.eye(3).reshape(-1).tolist()
        info.p = np.column_stack((matrix, np.zeros(3))).reshape(-1).tolist()
        self._publishers[(name, 'info')].publish(info)

    def close(self):
        if self._livox is not None:
            self._livox.close()
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        if self.context.ok():
            self.context.shutdown()
