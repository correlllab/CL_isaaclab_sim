# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0     
"""
public base scene configuration module
provides reusable scene element configurations, such as tables, objects, ground, lights, etc.
"""
import isaaclab.sim as sim_utils
from isaaclab.assets import  AssetBaseCfg, RigidObjectCfg
from isaaclab.sensors import CameraCfg

from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg, UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from tasks.common_config import   CameraBaseCfg  
import os
project_root = os.environ.get("PROJECT_ROOT")
ROOM_SIZE_X = 5.0
ROOM_SIZE_Y = 5.0
ROOM_HEIGHT = 3.0
WALL_THICKNESS = 0.1
@configclass
class TableRedGreenYellowBlockSceneCfg(InteractiveSceneCfg): # inherit from the interactive scene configuration class
    """object table scene configuration class
    defines a complete scene containing robot, object, table, etc.
    """
      # 1. room wall configuration - simplified configuration to avoid rigid body property conflicts

    floor = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Floor",
        spawn=sim_utils.CuboidCfg(
            size=(ROOM_SIZE_X+WALL_THICKNESS, ROOM_SIZE_Y+WALL_THICKNESS, WALL_THICKNESS),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                rigid_body_enabled=False,
             ),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.3, 0.3, 0.3),
            ),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0.0, 0.0, -WALL_THICKNESS),
        ),
    )


    #roof = AssetBaseCfg(
    #    prim_path="{ENV_REGEX_NS}/Roof",
    #    spawn=sim_utils.CuboidCfg(
    #        size=(ROOM_SIZE_X, ROOM_SIZE_Y, WALL_THICKNESS),
    #        collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
    #        rigid_props=sim_utils.RigidBodyPropertiesCfg(
    #            rigid_body_enabled=False,
    #         ),
    #        visual_material=sim_utils.PreviewSurfaceCfg(
    #            diffuse_color=(0.3, 0.2, 0.5),
    #        ),
    #    ),
    #    init_state=AssetBaseCfg.InitialStateCfg(
    #        pos=(0.0, 0.0, ROOM_HEIGHT),
    #    ),
    #)
    wall_px = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Wall1",
        spawn=sim_utils.CuboidCfg(
            size=(WALL_THICKNESS, ROOM_SIZE_Y, ROOM_HEIGHT),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                rigid_body_enabled=False,
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.244, 0.143, 0.177),
            ),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(ROOM_SIZE_X / 2 - WALL_THICKNESS / 2, 0.0, ROOM_HEIGHT / 2),
        ),
    )

    wall_nx = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Wall2",
        spawn=sim_utils.CuboidCfg(
            size=(WALL_THICKNESS, ROOM_SIZE_Y, ROOM_HEIGHT),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                rigid_body_enabled=False,
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.244, 0.198, 0.177),
            ),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(-ROOM_SIZE_X / 2 + WALL_THICKNESS / 2, 0.0, ROOM_HEIGHT / 2),
        ),
    )
    wall_py = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Wall3",
        spawn=sim_utils.CuboidCfg(
            size=(ROOM_SIZE_X, WALL_THICKNESS, ROOM_HEIGHT),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                rigid_body_enabled=False,
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.244, 0.198, 0.54),
            ),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0.0, ROOM_SIZE_Y / 2 - WALL_THICKNESS / 2, ROOM_HEIGHT / 2),
        ),
    )

    wall_ny = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Wall4",
        spawn=sim_utils.CuboidCfg(
            size=(ROOM_SIZE_X, WALL_THICKNESS, ROOM_HEIGHT),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                rigid_body_enabled=False,
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.0, 0.109, 0.0),
            ),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0.0, -ROOM_SIZE_Y / 2 + WALL_THICKNESS / 2, ROOM_HEIGHT / 2),
        ),
    )

    # 1. table configuration
    packing_table = AssetBaseCfg(
        prim_path="/World/envs/env_.*/PackingTable",    # table in the scene
        init_state=AssetBaseCfg.InitialStateCfg(pos=[1.00,-1.1,-0.1],   # initial position [x, y, z]
                                                rot=[1.0, 0.0, 0.0, 0.0]), # initial rotation [x, y, z, w]
        spawn=UsdFileCfg(
            usd_path=f"/workspace/hams/isaac/mateo_ws/CL_Assets/isaac_assets/objects/table_with_yellowbox.usd",    # table model file
            # rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),    # set to kinematic object
        ),
    )

    packing_table2 = AssetBaseCfg(
        prim_path="/World/envs/env_.*/PackingTable2",    # table in the scene
        init_state=AssetBaseCfg.InitialStateCfg(pos=[1.00,1.1,-0.1],   # initial position [x, y, z]
                                                rot=[0.0, 0.0, 0.0, 1.0]), # initial rotation [x, y, z, w]
        spawn=UsdFileCfg(
            usd_path=f"/workspace/hams/isaac/mateo_ws/CL_Assets/isaac_assets/objects/table_with_yellowbox.usd",    # table model file
            # rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),    # set to kinematic object
        ),
    )
    # Object
    red_block = RigidObjectCfg(
        prim_path="/World/envs/env_.*/Red_block",
        init_state=RigidObjectCfg.InitialStateCfg(
            #pos= [-4.1, -4.08, 0.84],
            pos= [-0.07749+1.00, -0.48792-0.4, 0.92775],
            rot=[1, 0, 0, 0]
        ),
        spawn=sim_utils.CuboidCfg(
            size=(0.05, 0.05, 0.05),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=True,
                retain_accelerations=False
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=True,
                contact_offset=0.01,
                rest_offset=0.0
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(1.0, 0.0, 0.0), metallic=0
            ),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                friction_combine_mode="max",
                restitution_combine_mode="min",
                static_friction=10,
                dynamic_friction=0.5,
                restitution=0.0,
            ),
        ),
    )

    yellow_block = RigidObjectCfg(
        prim_path="/World/envs/env_.*/Yellow_block",
        init_state=RigidObjectCfg.InitialStateCfg(
            #pos= [-4.25, -4.05, 0.84],
            pos= [0.15996+1.00, -0.61763-0.4, 0.92775],
            rot=[1, 0, 0, 0]
        ),
        spawn=sim_utils.CuboidCfg(
            size=(0.05, 0.05, 0.05),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=True,
                retain_accelerations=False
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=True,
                contact_offset=0.01,
                rest_offset=0.0
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(1.0, 1.0, 0.0), metallic=0
            ),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                friction_combine_mode="max",
                restitution_combine_mode="min",
                static_friction=10,
                dynamic_friction=0.5,
                restitution=0.0,
            ),
        ),
    )
    green_block = RigidObjectCfg(
        prim_path="/World/envs/env_.*/Green_block",
        init_state=RigidObjectCfg.InitialStateCfg(
            #pos=  [-4.18, -4.12, 0.84] ,
            pos=  [0.27413+1.00, -0.55893-0.4, 0.92775] ,
            rot=[1, 0, 0, 0]
        ),
        spawn=sim_utils.CuboidCfg(
            size=(0.05, 0.05, 0.05),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=True,
                retain_accelerations=False
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=True,
                contact_offset=0.01,
                rest_offset=0.0
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.0, 1.0, 0.0), metallic=0
            ),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                friction_combine_mode="max",
                restitution_combine_mode="min",
                static_friction=10,
                dynamic_friction=0.5,
                restitution=0.0,
            ),
        ),
    )

    pink_block = RigidObjectCfg(
        prim_path="/World/envs/env_.*/Pink_block",
        init_state=RigidObjectCfg.InitialStateCfg(
            #pos= [-4.1, -4.08, 0.84],
            pos= [0.06608+1.00, 0.47518+0.4, 0.92775],
            rot=[1, 0, 0, 0]
        ),
        spawn=sim_utils.CuboidCfg(
            size=(0.05, 0.05, 0.05),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=True,
                retain_accelerations=False
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=True,
                contact_offset=0.01,
                rest_offset=0.0
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.255, 0.156, 0.242), metallic=0
            ),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                friction_combine_mode="max",
                restitution_combine_mode="min",
                static_friction=10,
                dynamic_friction=0.5,
                restitution=0.0,
            ),
        ),
    )

    orange_block = RigidObjectCfg(
        prim_path="/World/envs/env_.*/Orange_block",
        init_state=RigidObjectCfg.InitialStateCfg(
            #pos= [-4.25, -4.05, 0.84],
            pos= [-0.11345+1.00, 0.57683+0.4, 0.92775],
            rot=[1, 0, 0, 0]
        ),
        spawn=sim_utils.CuboidCfg(
            size=(0.05, 0.05, 0.05),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=True,
                retain_accelerations=False
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=True,
                contact_offset=0.01,
                rest_offset=0.0
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.255, 0.156, 0.0), metallic=0
            ),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                friction_combine_mode="max",
                restitution_combine_mode="min",
                static_friction=10,
                dynamic_friction=0.5,
                restitution=0.0,
            ),
        ),
    )
    white_block = RigidObjectCfg(
        prim_path="/World/envs/env_.*/White_block",
        init_state=RigidObjectCfg.InitialStateCfg(
            #pos=  [-4.18, -4.12, 0.84] ,
            pos=  [-0.308+1.0, 0.43724+0.4, 0.92775] ,
            rot=[1, 0, 0, 0]
        ),
        spawn=sim_utils.CuboidCfg(
            size=(0.05, 0.05, 0.05),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=True,
                retain_accelerations=False
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=True,
                contact_offset=0.01,
                rest_offset=0.0
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(1.0, 1.0, 1.0), metallic=0
            ),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                friction_combine_mode="max",
                restitution_combine_mode="min",
                static_friction=10,
                dynamic_friction=0.5,
                restitution=0.0,
            ),
        ),
    )


    battery = AssetBaseCfg(
        prim_path="/World/envs/env_.*/Battery",
        spawn=UsdFileCfg(
            usd_path="/workspace/hams/isaac/mateo_ws/CL_Assets/isaac_assets/objects/battery.usd",
            scale=(1.0, 1.0, 1.0),
        ),
    )
    # Ground plane
    # 3. ground configuration
    #ground = AssetBaseCfg(
    #    prim_path="/World/GroundPlane",    # ground in the scene
    #    spawn=GroundPlaneCfg(color=(0.2, 0.2, 0.2)),
    #    init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0))
    #    # ground configuration
    #)

    # Lights
    # 4. light configuration
    light = AssetBaseCfg(
        prim_path="/World/light",   # light in the scene
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), # light color (white)
                                     intensity=1000.0),    # light intensity
    )


    world_camera = CameraBaseCfg.get_camera_config(prim_path="/World/PerspectiveCamera",
                                                    pos_offset=(0.0, -2.3, 1.5),
                                                    rot_offset=(-0.707, 0.707, 0.0, 0.0))

