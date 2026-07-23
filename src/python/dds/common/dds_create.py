# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
from .dds_master import dds_manager

def create_dds_objects(args_cli,env):
    publish_names = []
    subscribe_names = []
    if args_cli.robot_type=="h1_2":
        from ..specialized.h12_robot_dds import H12RobotDDS
        h12_robot = H12RobotDDS()
        dds_manager.register_object("h1_2", h12_robot)
        publish_names.append("h1_2")
        subscribe_names.append("h1_2")
    if args_cli.enable_inspire_dds:
        from ..specialized.inspire_dds import InspireDDS
        inspire = InspireDDS()
        dds_manager.register_object("inspire", inspire)
        publish_names.append("inspire")
        subscribe_names.append("inspire")

    dds_manager.start_publishing(publish_names)
    dds_manager.start_subscribing(subscribe_names)
    return dds_manager

