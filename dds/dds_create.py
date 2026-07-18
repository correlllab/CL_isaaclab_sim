# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
from dds.dds_master import dds_manager

def create_dds_objects(args_cli,env):
    publish_names = []
    subscribe_names = []
    if args_cli.robot_type=="h1_2":
        from dds.g1_robot_dds import G1RobotDDS
        g1_robot = G1RobotDDS()
        dds_manager.register_object("g129", g1_robot)
        publish_names.append("g129")
        subscribe_names.append("g129")
    if args_cli.enable_inspire_dds:
        from dds.inspire_dds import InspireDDS
        inspire = InspireDDS()
        dds_manager.register_object("inspire", inspire)
        publish_names.append("inspire")
        subscribe_names.append("inspire")
    #from dds.rewards_dds import RewardsDDS
    #rewards_dds = RewardsDDS(env,args_cli.task)
    #dds_manager.register_object("rewards", rewards_dds)
    #publish_names.append("rewards")

    dds_manager.start_publishing(publish_names)
    dds_manager.start_subscribing(subscribe_names)
    return dds_manager

