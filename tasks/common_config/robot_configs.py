# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
"""
public robot configuration
include the basic configuration for different robots, support scene-specific parameter customization
support different robot variants: with/without waist joint, different finger configurations
"""

from isaaclab.assets import ArticulationCfg
from isaaclab.utils import configclass
from robots.unitree import H12_CFG_WITH_INSPIRE_HAND
from typing import Optional, Dict, Tuple, Literal


@configclass
class RobotJointTemplates:
    """robot joint template collection
    
    provide different types of joint configuration templates
    """
    
    @classmethod
    def get_leg_joints(cls) -> Dict[str, float]:
        """get the default position of the leg joints"""
        return {
            # left leg joint - locked in standing position
            "left_hip_pitch_joint": 0.0,
            "left_hip_roll_joint": 0.0,
            "left_hip_yaw_joint": 0.0,
            "left_knee_joint": 0.0,
            "left_ankle_pitch_joint": 0.0,
            "left_ankle_roll_joint": 0.0,
            
            # right leg joint - locked in standing position
            "right_hip_pitch_joint": 0.0,
            "right_hip_roll_joint": 0.0,
            "right_hip_yaw_joint": 0.0,
            "right_knee_joint": 0.0,
            "right_ankle_pitch_joint": 0.0,
            "right_ankle_roll_joint": 0.0,
        }
    
    @classmethod
    def get_waist_joints(cls, include_waist: bool = True) -> Dict[str, float]:
        """get the position of the waist joints
        
        Args:
            include_waist: whether to include the waist joint
            
        Returns:
            waist joint position dictionary, if not included, return an empty dictionary
        """
        if not include_waist:
            return {}
        
        return {
            "waist_yaw_joint": 0.0,
            "waist_roll_joint": 0.0,
            "waist_pitch_joint": 0.0,
        }
    
    @classmethod
    def get_arm_joints(cls) -> Dict[str, float]:
        """get the default position of the arm joints"""
        return {
            # left arm joint
            "left_shoulder_pitch_joint": 0.0,
            "left_shoulder_roll_joint": 0.0,
            "left_shoulder_yaw_joint": 0.0,
            "left_elbow_joint": 0.0,
            "left_wrist_roll_joint": 0.0,
            "left_wrist_pitch_joint": 0.0,
            "left_wrist_yaw_joint": 0.0,
            
            # right arm joint
            "right_shoulder_pitch_joint": 0.0,
            "right_shoulder_roll_joint": 0.0,
            "right_shoulder_yaw_joint": 0.0,
            "right_elbow_joint": 0.0,
            "right_wrist_roll_joint": 0.0,
            "right_wrist_pitch_joint": 0.0,
            "right_wrist_yaw_joint": 0.0,
        }
    
    @classmethod
    def get_hand_joints(cls, hand_type="inspire") -> Dict[str, float]:
        """get the default position of the hand joints
        
        Returns:
            hand joint position dictionary
        """
        if hand_type == "inspire":
            return {
            # fingers joints
            "L_index_proximal_joint": 0.0,
            "L_index_intermediate_joint": 0.0,
            "L_middle_proximal_joint": 0.0,
            "L_middle_intermediate_joint": 0.0,
            "L_pinky_proximal_joint":0.0,
            "L_pinky_intermediate_joint":0.0,
            "L_ring_proximal_joint":0.0,
            "L_ring_intermediate_joint":0.0,
            "L_thumb_proximal_yaw_joint":0.0,
            "L_thumb_proximal_pitch_joint":0.0,
            "L_thumb_intermediate_joint":0.0,
            "L_thumb_distal_joint":0.0,

            "R_index_proximal_joint": 0.0,
            "R_index_intermediate_joint": 0.0,
            "R_middle_proximal_joint": 0.0,
            "R_middle_intermediate_joint": 0.0,
            "R_pinky_proximal_joint":0.0,
            "R_pinky_intermediate_joint":0.0,
            "R_ring_proximal_joint":0.0,
            "R_ring_intermediate_joint":0.0,
            "R_thumb_proximal_yaw_joint":0.0,
            "R_thumb_proximal_pitch_joint":0.0,
            "R_thumb_intermediate_joint":0.0,
            "R_thumb_distal_joint":0.0,
            }
        else:
            raise ValueError(f"Unsupported hand type: {hand_type}. Supported: 'inspire', 'magpie(coming soon)'")


@configclass
class RobotBaseCfg:
    """H12 robot base configuration class
    
    provide the flexible configuration for G1 robot, support:
    - with/without waist joint
    - simple gripper/dexterous hand
    - basic articulation configuration
    - default joint position
    - scene-specific parameter customization
    """
    
    @classmethod
    def get_base_config(
        cls,
        prim_path: str = "/World/envs/env_.*/Robot",
        init_pos: Tuple[float, float, float] = (-0.15, 0.0, 0.744),
        init_rot: Tuple[float, float, float, float] = (0.7071, 0, 0, 0.7071),
        include_waist: bool = True,
        hand_type: Literal["gripper", "dex3", "inspire"] = "gripper",
        base_config = None,
        custom_joint_pos: Optional[Dict[str, float]] = None,
        is_have_hand: bool = True,
        update_default_joint_pos: bool = True,
        robot_type: Literal["h1_2"] = "h1_2",
    ) -> ArticulationCfg:
        """get the base configuration for G1 robot
        
        Args:
            prim_path: the path of the robot in the scene
            init_pos: initial position (x, y, z)
            init_rot: initial rotation quaternion (w, x, y, z)
            include_waist: whether to include the waist joint
            hand_type: hand type ("simple" or "dexterous")
            base_config: base robot configuration, default using G129_CFG_WITH_DEX1_WAIST_FIX
            custom_joint_pos: custom joint position dictionary, will override the default value
            
        Returns:
            ArticulationCfg: robot configuration
        """
        
        # use the default base configuration
        if base_config is None and robot_type == "h1_2":
            base_config = H12_CFG_WITH_INSPIRE_HAND
        
        if update_default_joint_pos:
            # build the complete default joint position
            default_joint_pos = {}
            # add the leg joints
            default_joint_pos.update(RobotJointTemplates.get_leg_joints())
            
            # add the waist joints (if enabled)
            
            # add the arm joints
            default_joint_pos.update(RobotJointTemplates.get_arm_joints())
            
            # add the hand joints
            if is_have_hand:
                default_joint_pos.update(RobotJointTemplates.get_hand_joints(hand_type))
        else:
            default_joint_pos = base_config.init_state.joint_pos.copy()
        
        # if the custom joint position is provided, merge it
        if custom_joint_pos:
            joint_pos = {**default_joint_pos, **custom_joint_pos}
        else:
            joint_pos = default_joint_pos
        
        # create the base configuration
        return base_config.replace(
            prim_path=prim_path,
            init_state=ArticulationCfg.InitialStateCfg(
                pos=init_pos,
                rot=init_rot,
                joint_pos=joint_pos,
                joint_vel={".*": 0.0}
            ),
        )


@configclass
class H12RobotPresets:
    """H1 robot preset configuration collection
    
    include the common robot configuration preset for different scenes, support different robot variants
    """
    
    @classmethod
    def h12_27dof_inspire_base_fix(cls,init_pos: Tuple[float, float, float] = (-0.15, 0.0, 0.76),
        init_rot: Tuple[float, float, float, float] = (0.7071, 0, 0, 0.7071)) -> ArticulationCfg:   
        return RobotBaseCfg.get_base_config(
            init_pos=init_pos,
            init_rot=init_rot,
            include_waist=True,
            hand_type="inspire",
            base_config=H12_CFG_WITH_INSPIRE_HAND,
            robot_type="h1_2"
        )


