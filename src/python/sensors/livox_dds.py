"""Pinned livox_ros_driver2 message schema on the ROS 2 DDS wire.

Direct CycloneDDS avoids loading ROS-generated CPython message extensions
from a different Python ABI into Isaac Sim's interpreter.
"""
from dataclasses import dataclass

from cyclonedds.idl import IdlStruct
from cyclonedds.idl import annotations as annotate
from cyclonedds.idl import types
from unitree_sdk2py.idl.builtin_interfaces.msg.dds_ import Time_
from unitree_sdk2py.idl.std_msgs.msg.dds_ import Header_


@dataclass
@annotate.final
@annotate.autoid("sequential")
class CustomPoint_(IdlStruct, typename="livox_ros_driver2.msg.dds_.CustomPoint_"):
    offset_time: types.uint32
    x: types.float32
    y: types.float32
    z: types.float32
    reflectivity: types.uint8
    tag: types.uint8
    line: types.uint8


@dataclass
@annotate.final
@annotate.autoid("sequential")
class CustomMsg_(IdlStruct, typename="livox_ros_driver2.msg.dds_.CustomMsg_"):
    header: Header_
    timebase: types.uint64
    point_num: types.uint32
    lidar_id: types.uint8
    rsvd: types.array[types.uint8, 3]
    points: types.sequence[CustomPoint_]


class LivoxPublisher:
    def __init__(self, domain):
        from cyclonedds.domain import DomainParticipant
        from cyclonedds.pub import DataWriter
        from cyclonedds.topic import Topic
        from cyclonedds.qos import Qos, Policy
        from cyclonedds.util import duration
        self.participant = DomainParticipant(domain)
        self.topic = Topic(self.participant, 'rt/livox/lidar', CustomMsg_)
        self.writer = DataWriter(self.participant, self.topic, Qos(
            Policy.Reliability.Reliable(max_blocking_time=duration(milliseconds=100)),
            Policy.History.KeepLast(5),
        ))

    def publish(self, stamp_sec, stamp_nanosec, points):
        sample = CustomMsg_(
            Header_(Time_(stamp_sec, stamp_nanosec), 'lidar_link'),
            stamp_sec * 1_000_000_000 + stamp_nanosec,
            len(points), 0, [0, 0, 0],
            [CustomPoint_(int(p['timestamp']), float(p['x']), float(p['y']), float(p['z']),
                          int(p['intensity']), int(p['tag']), int(p['line'])) for p in points],
        )
        self.writer.write(sample)

    def close(self):
        # CycloneDDS entities release their native handles when references end.
        self.writer = None
        self.topic = None
        self.participant = None
