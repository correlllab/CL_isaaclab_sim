# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
"""
public camera configuration
include the basic configuration for different types of cameras, support scene-specific parameter customization
"""

import isaaclab.sim as sim_utils
import math
import isaaclab.utils.math as isaac_math
from isaaclab.sensors import CameraCfg
from isaaclab.utils import configclass
import os


@configclass
class CameraBaseCfg:
    """camera base configuration class
    
    provide the default configuration for different types of cameras, support scene-specific parameter customization
    """
    
    @classmethod
    def get_camera_config(
        cls,
        prim_path: str = "/World/envs/env_.*/Robot/d435_link/front_cam",
        update_period: float = 0.02,
        height: int = 480,
        width: int =  640,
        focal_length: float = 7.6,
        focus_distance: float = 400.0,
        horizontal_aperture: float = 20.0,
        clipping_range: tuple = (0.1, 1.0e5),
        pos_offset: tuple = (0, 0.0, 0),
        rot_offset: tuple = (0.5, -0.5, 0.5, -0.5),
        data_types: list = None
    ) -> CameraCfg:
        """get the front camera configuration
        
        Args:
            prim_path: the path of the camera in the scene
            update_period: update period (seconds)
            height: image height (pixels)
            width: image width (pixels)
            focal_length: focal length
            focus_distance: focus distance
            horizontal_aperture: horizontal aperture
            clipping_range: clipping range (near clipping plane, far clipping plane)
            pos_offset: position offset (x, y, z)
            rot_offset: rotation offset quaternion
            data_types: data type list
            
        Returns:
            CameraCfg: camera configuration
        """
        if data_types is None:
            data_types = ["rgb"]

        return CameraCfg(
            prim_path=prim_path,
            update_period=update_period,
            height=height,
            width=width,
            data_types=data_types,
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=focal_length,
                focus_distance=focus_distance,
                horizontal_aperture=horizontal_aperture,
                clipping_range=clipping_range
            ),
            offset=CameraCfg.OffsetCfg(
                pos=pos_offset,
                rot=rot_offset,
                convention="ros"
            )
        )
    



@configclass
class CameraPresets:
    """camera preset configuration collection
    
    include the common camera configuration preset for different scenes
    """
    
#    world_camera = CameraBaseCfg.get_camera_config(prim_path="/World/PerspectiveCamera",
#                                                    pos_offset=(-4.4, -3.0, 1.8),
#                                                    rot_offset=( 2.54, 3.01, 0.0, 0.0))
#
#    cam_0 = CameraBaseCfg.get_camera_config(prim_path="/World/cam_0",
#                                                    pos_offset=(1.89933, 1.8901, 3.45455),
#                                                    rot_offset=(42.7, 0, 139.2))
#
#    cam_1 = CameraBaseCfg.get_camera_config(prim_path="/World/cam_1",
#                                                    pos_offset=(-1.7, 1.8901, 3.45455),
#                                                    rot_offset=(34.8, 23.6, 186.0))

    @classmethod
    def correll_camera_0(cls) -> CameraCfg:
        #return CameraBaseCfg.get_camera_config(prim_path="/World/PerspectiveCamera", pos_offset=(1.89933, 1.8901, 3.45455), rot_offset=(42.7, 0, 139.2))
        return CameraBaseCfg.get_camera_config(prim_path="/World/CorrellCam0", pos_offset=(1.3, 1.5, 3.45455), rot_offset=(-0.3882, -0.13878, -0.3194, -0.85324))


    @classmethod
    def correll_camera_1(cls) -> CameraCfg:
        return CameraBaseCfg.get_camera_config(prim_path="/World/CorrellCam1", pos_offset=(-1.7, 1.8901, 3.45455), rot_offset=(0.17955, -0.30253, 0.92954, -0.10995))


    @classmethod
    def correll_camera_2(cls) -> CameraCfg:
        return CameraBaseCfg.get_camera_config(prim_path="/World/CorrellCam2", pos_offset=(-1.7, -1.7, 3.45455), rot_offset=(0.30424, 0.09594, -0.30597, 0.897))


    @classmethod
    def g1_front_camera(cls) -> CameraCfg:
        """front camera configuration"""
        return CameraBaseCfg.get_camera_config()
    @classmethod
    def h12_front_camera(cls) -> CameraCfg:
        """front camera configuration"""
        return CameraBaseCfg.get_camera_config(prim_path = "/World/envs/env_.*/Robot/camera_link/front_cam")
    @classmethod
    def g1_world_camera(cls) -> CameraCfg:
        """front camera configuration"""
        return CameraBaseCfg.get_camera_config(prim_path="/World/envs/env_.*/Robot/d435_link/PerspectiveCamera_robot",
                                                    pos_offset=(-0.9, 0.0, 0.0),
                                                    rot_offset=( -0.51292,0.51292,-0.48674, 0.48674),
                                                    focal_length = 12,
                                                    horizontal_aperture=27)
    @classmethod
    def h12_world_camera(cls) -> CameraCfg:
        """front camera configuration"""
        return CameraBaseCfg.get_camera_config(prim_path="/World/envs/env_.*/Robot/camera_link/PerspectiveCamera_robot",
                                                    pos_offset=(-0.9, 0.0, 0.0),
                                                    rot_offset=( -0.51292,0.51292,-0.48674, 0.48674),
                                                    focal_length = 12,
                                                    horizontal_aperture=27)
    @classmethod
    def left_gripper_wrist_camera(cls) -> CameraCfg:
        """left wrist camera configuration"""
        return CameraBaseCfg.get_camera_config(
            prim_path="/World/envs/env_.*/Robot/left_hand_base_link/left_wrist_camera",
            height=480,
            width=640,
            update_period=0.02,
            data_types=["rgb"],
            focal_length=12,
            focus_distance=400.0,
            horizontal_aperture=20.0,
            clipping_range=(0.1, 1.0e5),
            pos_offset=(0.02541028, 0.045, 0.135),
            rot_offset=(-0.34202, 0.93969, 0, 0),
        )
    @classmethod
    def right_gripper_wrist_camera(cls) -> CameraCfg:
        """right wrist camera configuration"""
        return CameraBaseCfg.get_camera_config(
            prim_path="/World/envs/env_.*/Robot/right_hand_base_link/right_wrist_camera",
            height=480,
            width=640,
            update_period=0.02,
            data_types=["rgb"],
            focal_length=12,
            focus_distance=400.0,
            horizontal_aperture=20.0,
            clipping_range=(0.1, 1.0e5),
            pos_offset=(-0.02541028, 0.045, 0.135),
            rot_offset=(-0.34202, 0.93969, 0, 0),
        ) 
    @classmethod
    def left_dex3_wrist_camera(cls) -> CameraCfg:
        """left wrist camera configuration"""
        return CameraBaseCfg.get_camera_config(
            prim_path="/World/envs/env_.*/Robot/left_hand_camera_base_link/left_wrist_camera",
            height=480,
            width=640,
            update_period=0.02,
            data_types=["rgb"],
            focal_length=12.0,
            focus_distance=400.0,
            horizontal_aperture=20.0,
            clipping_range=(0.1, 1.0e5),
            pos_offset=(-0.04012, -0.07441 ,0.15711),
            rot_offset=(0.00539,0.86024,0.0424, 0.50809),
        )
    @classmethod
    def right_dex3_wrist_camera(cls) -> CameraCfg:
        """right wrist camera configuration"""
        return CameraBaseCfg.get_camera_config(
            prim_path="/World/envs/env_.*/Robot/right_hand_camera_base_link/right_wrist_camera",
            height=480,
            width=640,
            update_period=0.02,
            data_types=["rgb"],
            focal_length=12.0,
            focus_distance=400.0,
            horizontal_aperture=20.0,
            clipping_range=(0.1, 1.0e5),
            pos_offset=(-0.04012, 0.07441 ,0.15711),
            rot_offset=(0.00539,0.86024,0.0424, 0.50809),
        ) 
    
    @classmethod
    def left_inspire_wrist_camera(cls) -> CameraCfg:
        """left wrist camera configuration"""
        return CameraBaseCfg.get_camera_config(
            prim_path="/World/envs/env_.*/Robot/left_hand_camera_base_link/left_wrist_camera",
            height=480,
            width=640,
            update_period=0.02,
            data_types=["rgb", "distance_to_image_plane"],
            focal_length=12.0,
            focus_distance=400.0,
            horizontal_aperture=20.0,
            clipping_range=(0.1, 1.0e5),
            pos_offset=(-0.04012, -0.07441 ,0.15711),
            rot_offset=(0.00539,0.86024,0.0424, 0.50809),
        )
    @classmethod
    def right_inspire_wrist_camera(cls) -> CameraCfg:
        """right wrist camera configuration"""
        return CameraBaseCfg.get_camera_config(
            prim_path="/World/envs/env_.*/Robot/right_hand_camera_base_link/right_wrist_camera",
            height=480,
            width=640,
            update_period=0.02,
            data_types=["rgb", "distance_to_image_plane"],
            focal_length=12.0,
            focus_distance=400.0,
            horizontal_aperture=20.0,
            clipping_range=(0.1, 1.0e5),
            pos_offset=(-0.04012, 0.07441 ,0.15711),
            rot_offset=(0.00539,0.86024,0.0424, 0.50809),
        ) 
